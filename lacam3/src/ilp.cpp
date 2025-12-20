#include "../include/ilp.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

ILP::ILP(const Instance *_ins, DistTable *_D)
    : ins(_ins),
      N(ins->N),
      V_size(ins->G->size()),
      D(_D),
      env(nullptr),
      env_ready(false),
      candidates(),
      visited_solutions(),
      save_model(false), // For debugging only
      model_dir("ilp_models"),
      model_serial(0),
      log_timing(false),
      total_ms(0.0),
      last_ms(0.0),
      call_count(0)
{
  try {
    env = std::make_unique<GRBEnv>(true);
    env->set(GRB_IntParam_OutputFlag, 0);
    env->start();
    env_ready = true;
  } catch (GRBException &e) {
    std::cerr << "Gurobi init failed: " << e.getMessage() << std::endl;
    env_ready = false;
  }
}

ILP::~ILP() = default;

size_t ILP::StateKeyHasher::operator()(const StateKey &key) const
{
  size_t h = 0;
  for (const auto v : key) {
    h ^= std::hash<int>{}(v) + 0x9e3779b9 + (h << 6) + (h >> 2);
  }
  return h;
}

ILP::StateKey ILP::build_state_key(const Config &Q_from,
                                   const Config &Q_to) const
{
  StateKey key;
  key.reserve(N * 2);
  for (size_t i = 0; i < N; ++i) key.push_back(Q_from[i]->id);
  for (size_t i = 0; i < N; ++i)
    key.push_back(Q_to[i] ? Q_to[i]->id : -1);
  return key;
}

ILP::StateKey ILP::build_solution_key(const Config &Q_to) const
{
  StateKey key;
  key.reserve(N);
  for (size_t i = 0; i < N; ++i) key.push_back(Q_to[i]->id);
  return key;
}

bool ILP::init_model_from_config(const Config &Q_from)
{
  if (!env_ready) return false;
  candidates.assign(N, Vertices());
  for (size_t i = 0; i < N; ++i) {
    const auto &nbrs = Q_from[i]->neighbor;
    candidates[i].reserve(nbrs.size() + 1);
    for (auto v : nbrs) candidates[i].push_back(v);
    candidates[i].push_back(Q_from[i]);  // allow wait
  }
  return true;
}

bool ILP::set_new_config(const Config &Q_from, Config &Q_to)
{
  if (!env_ready) return false;
  if (!init_model_from_config(Q_from)) return false;

  try {
    const auto t_start = std::chrono::steady_clock::now();
    const auto state_key = build_state_key(Q_from, Q_to);
    GRBModel model(*env);
    std::vector<std::vector<GRBVar>> x(N);

    // create variables and per-agent constraints
    for (size_t i = 0; i < N; ++i) {
      const auto cand_sz = candidates[i].size();
      x[i].resize(cand_sz);
      GRBLinExpr assign = 0;

      int fixed_idx = -1;
      if (Q_to[i] != nullptr) {
        for (size_t k = 0; k < cand_sz; ++k) {
          if (candidates[i][k] == Q_to[i]) {
            fixed_idx = static_cast<int>(k);
            break;
          }
        }
        if (fixed_idx < 0) return false;
      }

      for (size_t k = 0; k < cand_sz; ++k) {
        const auto name = "x_" + std::to_string(i) + "_" + std::to_string(k);
        x[i][k] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, name);
        assign += x[i][k];
      }
      model.addConstr(assign == 1.0, "assign_" + std::to_string(i));
      if (fixed_idx >= 0) {
        model.addConstr(x[i][fixed_idx] == 1.0,
                        "fixed_" + std::to_string(i));
      }
    }

    // vertex conflict constraints
    std::vector<GRBLinExpr> vertex_sums(V_size);
    std::vector<int> vertex_counts(V_size, 0);
    for (size_t i = 0; i < N; ++i) {
      for (size_t k = 0; k < candidates[i].size(); ++k) {
        const int v_id = candidates[i][k]->id;
        vertex_sums[v_id] += x[i][k];
        vertex_counts[v_id] += 1;
      }
    }
    for (int v_id = 0; v_id < V_size; ++v_id) {
      if (vertex_counts[v_id] > 1) {
        model.addConstr(vertex_sums[v_id] <= 1.0,
                        "vertex_" + std::to_string(v_id));
      }
    }

    // edge conflict constraints (swap)
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = i + 1; j < N; ++j) {
        if (Q_from[i] == Q_from[j]) continue;
        int idx_i = -1;
        int idx_j = -1;
        for (size_t k = 0; k < candidates[i].size(); ++k) {
          if (candidates[i][k] == Q_from[j]) {
            idx_i = static_cast<int>(k);
            break;
          }
        }
        for (size_t k = 0; k < candidates[j].size(); ++k) {
          if (candidates[j][k] == Q_from[i]) {
            idx_j = static_cast<int>(k);
            break;
          }
        }
        if (idx_i >= 0 && idx_j >= 0) {
          model.addConstr(x[i][idx_i] + x[j][idx_j] <= 1.0,
                          "swap_" + std::to_string(i) + "_" +
                              std::to_string(j));
        }
      }
    }

    // exclude previously visited joint actions for the same state
    auto visited_it = visited_solutions.find(state_key);
    if (visited_it != visited_solutions.end()) {
      int cut_idx = 0;
      for (const auto &solution : visited_it->second) {
        GRBLinExpr nogood = 0;
        bool valid = true;
        for (size_t i = 0; i < N; ++i) {
          int match_idx = -1;
          for (size_t k = 0; k < candidates[i].size(); ++k) {
            if (candidates[i][k]->id == solution[i]) {
              match_idx = static_cast<int>(k);
              break;
            }
          }
          if (match_idx < 0) {
            valid = false;
            break;
          }
          nogood += x[i][match_idx];
        }
        if (valid) {
          model.addConstr(nogood <= static_cast<int>(N) - 1,
                          "nogood_" + std::to_string(cut_idx));
          ++cut_idx;
        }
      }
    }

    // objective: minimize total distance-to-go
    GRBLinExpr obj = 0;
    for (size_t i = 0; i < N; ++i) {
      for (size_t k = 0; k < candidates[i].size(); ++k) {
        obj += D->get(i, candidates[i][k]) * x[i][k];
      }
    }
    model.setObjective(obj, GRB_MINIMIZE);

    if (save_model) {
      std::error_code ec;
      std::filesystem::create_directories(model_dir, ec);
      const auto hash_val = StateKeyHasher{}(state_key);
      const auto path = model_dir + "/ilp_" + std::to_string(hash_val) + "_" +
                        std::to_string(model_serial++) + ".lp";
      try {
        model.write(path);
      } catch (GRBException &e) {
        std::cerr << "Gurobi model write failed: " << e.getMessage()
                  << std::endl;
      }
    }

    model.optimize();

    const int sol_count = model.get(GRB_IntAttr_SolCount);
    if (sol_count == 0) return false;

    // extract solution
    for (size_t i = 0; i < N; ++i) {
      bool assigned = false;
      for (size_t k = 0; k < candidates[i].size(); ++k) {
        if (x[i][k].get(GRB_DoubleAttr_X) > 0.5) {
          Q_to[i] = candidates[i][k];
          assigned = true;
          break;
        }
      }
      if (!assigned) return false;
    }

    auto solution_key = build_solution_key(Q_to);
    auto &solutions = visited_solutions[state_key];
    if (std::find(solutions.begin(), solutions.end(), solution_key) ==
        solutions.end()) {
      solutions.push_back(std::move(solution_key));
    }
    const auto t_end = std::chrono::steady_clock::now();
    last_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t_end - t_start)
            .count();
    total_ms += last_ms;
    ++call_count;
    if (log_timing) {
      std::cout << "ILP elapsed: " << last_ms << "ms (call " << call_count
                << ", total " << total_ms << "ms)" << std::endl;
    }
    return true;
  } catch (GRBException &e) {
    std::cerr << "Gurobi error: " << e.getMessage() << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unknown error while solving ILP." << std::endl;
    return false;
  }
}
