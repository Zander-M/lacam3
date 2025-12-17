// ILP config generator 

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "scatter.hpp"
#include "utils.hpp"
#include "gurobi_c++.h"
#include <memory>

struct ILP{
  const Instance *ins;

  // solver utils
  const int N; //number of agents
  const int V_size;
  DistTable *D;

  // ILP specific
  std::unique_ptr<GRBEnv> env;
  bool env_ready;
  std::vector<Vertices> candidates;
  using StateKey = std::vector<int>;
  struct StateKeyHasher {
    size_t operator()(const StateKey &key) const;
  };
  std::unordered_map<StateKey, std::vector<StateKey>, StateKeyHasher>
      visited_solutions;
  bool save_model;
  std::string model_dir;
  int model_serial;
  bool log_timing;
  double total_ms;
  double last_ms;
  int call_count;

  ILP(const Instance *_ins, DistTable *_D);
  ~ILP();
  bool set_new_config(const Config &Q_from, Config &Q_to);
  bool init_model_from_config(const Config &Q_from); // Create ILP problem
  StateKey build_state_key(const Config &Q_from, const Config &Q_to) const;
  StateKey build_solution_key(const Config &Q_to) const;
};
