#ifndef TRIG_QUAD_RULE_H
#define TRIG_QUAD_RULE_H

#include <Eigen/Dense>

namespace green {

// number: 3n^2
// q: size 2*number
// w: size number
void sym_trig_quad_rule(const size_t number,
                        Eigen::Matrix2Xd &q,
                        Eigen::VectorXd  &w);

}
#endif
