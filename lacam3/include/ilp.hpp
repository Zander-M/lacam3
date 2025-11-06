// ILP config generator 

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "scatter.hpp"
#include "utils.hpp"
#include "gurobi_c++.h"

struct ILP{
  // TODO: Implement this
  const Instance *ins;

  // solver utils
  const int N; //number of agents
  const int V_size;
  DistTable *D;

  // ILP specific
  ILP(const Instance *_ins, DistTable *_D);
  ~ILP();
  bool set_new_config(const Config &Q_from, Config &Q_to);
  bool init_model_from_config(const Config &Q_form);
};