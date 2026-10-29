#include <easyfly_trajectory_optimizer/plan.hpp>
#include <iostream>
int main()
{
  namespace et = easyfly::trajectory;
  et::Request r;
  r.grid.resolution = .2; r.grid.origin = {-2., -3., 0.};
  r.grid.size = {30, 30, 18};
  r.grid.data.assign(30 * 30 * 18, 0);
  r.start = {0., 0., 1.3}; r.velocity = {0., 0., 0.}; r.goal = {2., 0., 1.3};
  auto p = et::optimize(r, et::Steady::now() + std::chrono::seconds(2));
  if ((p.trajectory.getPos(p.trajectory.getTotalDuration()) - r.goal).norm() > 1e-5) {return 1;}
  std::cout << p.trajectory.getPieceNum() << " pieces, max speed " << p.maximum_speed << '\n';
}
