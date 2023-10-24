#include "global.h"
#include "constants.h"
#include "node_pool.h"
#include "pqueue.h"
using namespace global;
uint32_t statis::subopt_expd = 0;
uint32_t statis::subopt_gen = 0;
uint32_t statis::scan_cnt = 0;
uint32_t statis::subopt_insert = 0;
string global::alg = "";
uint32_t statis::prunable = 0;
vector<warthog::cost_t> statis::dist = vector<warthog::cost_t>();
vector<statis::Log> statis::logs = vector<statis::Log>();
warthog::problem_instance* query::pi = nullptr;
warthog::mem::node_pool* global::nodepool = nullptr;
uint32_t query::startid = warthog::INF32;
uint32_t query::goalid = warthog::INF32;
warthog::cost_t query::cur_diag_gval = warthog::INFTY;
warthog::gridmap* query::map = nullptr;
warthog::pqueue_min* query::open = nullptr;
uint32_t global::query::jump_step = 0;
warthog::solution* global::sol = nullptr;
