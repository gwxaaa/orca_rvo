
#include "Obstacle.h"

namespace RVO {
// Obstacle::Obstacle()
//     : next_(NULL), previous_(NULL), id_(0U), isConvex_(false) {}
Obstacle::Obstacle(const Vector2 &point,  Obstacle *next, bool isConvex)
:
next_(next),
isConvex_(isConvex)
 {}
Obstacle::~Obstacle() {}
} /* namespace RVO */
