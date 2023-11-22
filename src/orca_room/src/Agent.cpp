
#include "Agent.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "Neighbor.h"
#include "Obstacle.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "ModelSubPub.h"
namespace RVO
{
  class neighbor;
  namespace
  {
    bool linearProgram1(const std::vector<Line> &lines, std::size_t lineNo,
                        float radius, const Vector2 &optVelocity, bool directionOpt,
                        Vector2 &result)
    { /* NOLINT(runtime/references) */
      const float dotProduct = lines[lineNo].point * lines[lineNo].direction;
      const float discriminant =
          dotProduct * dotProduct + radius * radius - absSq(lines[lineNo].point);
      if (discriminant < 0.0F)
      {
        /* Max speed circle fully invalidates line lineNo. */
        return false;
      }
      const float sqrtDiscriminant = std::sqrt(discriminant);
      float tLeft = -dotProduct - sqrtDiscriminant;
      float tRight = -dotProduct + sqrtDiscriminant;

      for (std::size_t i = 0U; i < lineNo; ++i)
      {
        const float denominator = det(lines[lineNo].direction, lines[i].direction);
        const float numerator =
            det(lines[i].direction, lines[lineNo].point - lines[i].point);
        if (std::fabs(denominator) <= RVO_EPSILON)
        {
          /* Lines lineNo and i are (almost) parallel. */
          if (numerator < 0.0F)
          {
            return false;
          }
          continue;
        }
        const float t = numerator / denominator;
        if (denominator >= 0.0F)
        {
          /* Line i bounds line lineNo on the right. */
          tRight = std::min(tRight, t);
        }
        else
        {
          /* Line i bounds line lineNo on the left. */
          tLeft = std::max(tLeft, t);
        }

        if (tLeft > tRight)
        {
          return false;
        }
      }
      if (directionOpt)
      {
        /* Optimize direction. */
        if (optVelocity * lines[lineNo].direction > 0.0F)
        {
          /* Take right extreme. */
          result = lines[lineNo].point + tRight * lines[lineNo].direction;
        }
        else
        {
          /* Take left extreme. */
          result = lines[lineNo].point + tLeft * lines[lineNo].direction;
        }
      }
      else
      {
        /* Optimize closest point. */
        const float t =
            lines[lineNo].direction * (optVelocity - lines[lineNo].point);
        if (t < tLeft)
        {
          result = lines[lineNo].point + tLeft * lines[lineNo].direction;
        }
        else if (t > tRight)
        {
          result = lines[lineNo].point + tRight * lines[lineNo].direction;
        }
        else
        {
          result = lines[lineNo].point + t * lines[lineNo].direction;
        }
      }
      return true;
    }

    std::size_t linearProgram2(const std::vector<Line> &lines, float radius,
                               const Vector2 &optVelocity, bool directionOpt,
                               Vector2 &result)
    { /* NOLINT(runtime/references) */
      if (directionOpt)
      {
        result = optVelocity * radius;
      }
      else if (absSq(optVelocity) > radius * radius)
      {
        /* Optimize closest point and outside circle. */
      }
      else
      {
        /* Optimize closest point and inside circle. */
        result = optVelocity;
      }
      for (std::size_t i = 0U; i < lines.size(); ++i)
      {
        if (det(lines[i].direction, lines[i].point - result) > 0.0F)
        {
          /* Result does not satisfy constraint i. Compute new optimal result. */
          const Vector2 tempResult = result;

          if (!linearProgram1(lines, i, radius, optVelocity, directionOpt,
                              result))
          {
            result = tempResult;
            return i;
          }
        }
      }
      return lines.size();
    }
    void linearProgram3(const std::vector<Line> &lines, std::size_t numObstLines,
                        std::size_t beginLine, float radius,
                        Vector2 &result)
    { /* NOLINT(runtime/references) */
      float distance = 0.0F;

      for (std::size_t i = beginLine; i < lines.size(); ++i)
      {
        if (det(lines[i].direction, lines[i].point - result) > distance)
        {
          /* Result does not satisfy constraint of line i. */
          std::vector<Line> projLines(
              lines.begin(),
              lines.begin() + static_cast<std::ptrdiff_t>(numObstLines));
          for (std::size_t j = numObstLines; j < i; ++j)
          {
            Line line;

            const float determinant = det(lines[i].direction, lines[j].direction);

            if (std::fabs(determinant) <= RVO_EPSILON)
            {
              /* Line i and line j are parallel. */
              if (lines[i].direction * lines[j].direction > 0.0F)
              {
                /* Line i and line j point in the same direction. */
                continue;
              }

              /* Line i and line j point in opposite direction. */
              line.point = 0.5F * (lines[i].point + lines[j].point);
            }
            else
            {
              line.point = lines[i].point + (det(lines[j].direction,
                                                 lines[i].point - lines[j].point) /
                                             determinant) *
                                                lines[i].direction;
            }
            line.direction = normalize(lines[j].direction - lines[i].direction);
            projLines.push_back(line);
          }

          const Vector2 tempResult = result;

          if (linearProgram2(
                  projLines, radius,
                  Vector2(-lines[i].direction.y(), lines[i].direction.x()), true,
                  result) < projLines.size())
          {
            result = tempResult;
          }
          distance = det(lines[i].direction, lines[i].point - result);
        }
      }
    }
  } /* namespace */
  Agent::Agent(const Vector2 &agentPosition, const Vector2 &agentVelocity, const Vector2 &goalPosition, double time,
               double maxSpeed_, double neighborDistance_, double timeHorizon_, const std::vector<gazebo_msgs::ModelState> other_models_states,
               double radius_)
      : time(time),
        maxSpeed_(maxSpeed_),
        neighborDist_(neighborDist_),
        radius_(radius_),
        timeHorizon_(timeHorizon_),
        timeHorizonObst_(timeHorizonObst_)
  {
  }
  Agent::~Agent() {}
  void Agent::computeNeighbors(const Neighbor *neighbor)
  {
    obstacleNeighbors_.clear();
    agentNeighbors_.clear();
    const std::vector<RVO::Agent *> &agentNeighbors_ = neighbor->getAgentNeighbors();
    // 获取障碍物邻居信息
    const std::vector<RVO::Obstacle *> &obstacleNeighbors_ = neighbor->getObstacleNeighbors();
  }
  // void Agent::computeNewVelocity(float timeStep)
  Vector2 Agent::computeNewVelocity(const Vector2 &agentPosition, const Vector2 &agentVelocity,
                                    const Vector2 &goalPosition,
                                    const std::vector<RVO::Agent *> &agentNeighbors_,
                                    const std::vector<RVO::Obstacle *> &obstacleNeighbors_,
                                    double time)
  {
    Vector2 position_(agentPosition);
    Vector2 velocity_(agentVelocity);
    double velocityX1 = (goalPosition.x() - agentPosition.x()) * 0.1;
    double velocityY1 = (goalPosition.y() - agentPosition.y()) * 0.1;
    Vector2 prefVelocity_(velocityX1, velocityY1);
    orcaLines_.clear();
    const float invTimeHorizonObst = 1.0 / timeHorizonObst_;

    /* Create obstacle ORCA lines. */
    for (std::size_t i = 0U; i < obstacleNeighbors_.size(); ++i)
    {
      const RVO::Obstacle *obstacle1 = obstacleNeighbors_[i];
      const Obstacle *obstacle2 = obstacle1->next_;
      const Vector2 relativePosition1 = obstacle1->point_ - position_;
      const Vector2 relativePosition2 = obstacle2->point_ - position_;

      /* Check if velocity obstacle of obstacle is already taken care of by
       * previously constructed obstacle ORCA lines. */
      bool alreadyCovered = false;
      for (std::size_t j = 0U; j < orcaLines_.size(); ++j)
      {
        if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point,
                orcaLines_[j].direction) -
                    invTimeHorizonObst * radius_ >=
                -RVO_EPSILON &&
            det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point,
                orcaLines_[j].direction) -
                    invTimeHorizonObst * radius_ >=
                -RVO_EPSILON)
        {
          alreadyCovered = true;
          break;
        }
      }
      if (alreadyCovered)
      {
        continue;
      }
      /* Not yet covered. Check for collisions. */
      const float distSq1 = absSq(relativePosition1);
      const float distSq2 = absSq(relativePosition2);
      const float radiusSq = radius_ * radius_;
      const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;
      const float s =
          (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
      const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);
      Line line;
      if (s < 0.0F && distSq1 <= radiusSq)
      {
        /* Collision with left vertex. Ignore if non-convex. */
        if (obstacle1->isConvex_)
        {
          line.point = Vector2(0.0F, 0.0F);
          line.direction =
              normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
          orcaLines_.push_back(line);
        }
        continue;
      }
      if (s > 1.0F && distSq2 <= radiusSq)
      {
        /* Collision with right vertex. Ignore if non-convex or if it will be
         * taken care of by neighoring obstace */
        if (obstacle2->isConvex_ &&
            det(relativePosition2, obstacle2->direction_) >= 0.0F)
        {
          line.point = Vector2(0.0F, 0.0F);
          line.direction =
              normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
          orcaLines_.push_back(line);
        }
        continue;
      }
      if (s >= 0.0F && s <= 1.0F && distSqLine <= radiusSq)
      {
        /* Collision with obstacle segment. */
        line.point = Vector2(0.0F, 0.0F);
        line.direction = -obstacle1->direction_;
        orcaLines_.push_back(line);
        continue;
      }
      /* No collision. Compute legs. When obliquely viewed, both legs can come
       * from a single vertex. Legs extend cut-off line when nonconvex vertex. */
      Vector2 leftLegDirection;
      Vector2 rightLegDirection;
      if (s < 0.0F && distSqLine <= radiusSq)
      {
        /* Obstacle viewed obliquely so that left vertex defines velocity
         * obstacle. */
        if (!obstacle1->isConvex_)
        {
          /* Ignore obstacle. */
          continue;
        }
        obstacle2 = obstacle1;
        const float leg1 = std::sqrt(distSq1 - radiusSq);
        leftLegDirection =
            Vector2(
                relativePosition1.x() * leg1 - relativePosition1.y() * radius_,
                relativePosition1.x() * radius_ + relativePosition1.y() * leg1) /
            distSq1;
        rightLegDirection =
            Vector2(
                relativePosition1.x() * leg1 + relativePosition1.y() * radius_,
                -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) /
            distSq1;
      }
      else if (s > 1.0F && distSqLine <= radiusSq)
      {
        /* Obstacle viewed obliquely so that right vertex defines velocity
         * obstacle. */
        if (!obstacle2->isConvex_)
        {
          /* Ignore obstacle. */
          continue;
        }
        obstacle1 = obstacle2;

        const float leg2 = std::sqrt(distSq2 - radiusSq);
        leftLegDirection =
            Vector2(
                relativePosition2.x() * leg2 - relativePosition2.y() * radius_,
                relativePosition2.x() * radius_ + relativePosition2.y() * leg2) /
            distSq2;
        rightLegDirection =
            Vector2(
                relativePosition2.x() * leg2 + relativePosition2.y() * radius_,
                -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) /
            distSq2;
      }
      else
      {
        /* Usual situation. */
        if (obstacle1->isConvex_)
        {
          const float leg1 = std::sqrt(distSq1 - radiusSq);
          leftLegDirection = Vector2(relativePosition1.x() * leg1 -
                                         relativePosition1.y() * radius_,
                                     relativePosition1.x() * radius_ +
                                         relativePosition1.y() * leg1) /
                             distSq1;
        }
        else
        {
          /* Left vertex non-convex; left leg extends cut-off line. */
          leftLegDirection = -obstacle1->direction_;
        }

        if (obstacle2->isConvex_)
        {
          const float leg2 = std::sqrt(distSq2 - radiusSq);
          rightLegDirection = Vector2(relativePosition2.x() * leg2 +
                                          relativePosition2.y() * radius_,
                                      -relativePosition2.x() * radius_ +
                                          relativePosition2.y() * leg2) /
                              distSq2;
        }
        else
        {
          /* Right vertex non-convex; right leg extends cut-off line. */
          rightLegDirection = obstacle1->direction_;
        }
      }
      /* Legs can never point into neighboring edge when convex vertex, take
       * cutoff-line of neighboring edge instead. If velocity projected on
       * "foreign" leg, no constraint is added. */
      const Obstacle *const leftNeighbor = obstacle1->previous_;
      bool isLeftLegForeign = false;
      bool isRightLegForeign = false;

      if (obstacle1->isConvex_ &&
          det(leftLegDirection, -leftNeighbor->direction_) >= 0.0F)
      {
        /* Left leg points into obstacle. */
        leftLegDirection = -leftNeighbor->direction_;
        isLeftLegForeign = true;
      }
      if (obstacle2->isConvex_ &&
          det(rightLegDirection, obstacle2->direction_) <= 0.0F)
      {
        /* Right leg points into obstacle. */
        rightLegDirection = obstacle2->direction_;
        isRightLegForeign = true;
      }
      /* Compute cut-off centers. */
      const Vector2 leftCutoff =
          invTimeHorizonObst * (obstacle1->point_ - position_);
      const Vector2 rightCutoff =
          invTimeHorizonObst * (obstacle2->point_ - position_);
      const Vector2 cutoffVector = rightCutoff - leftCutoff;
      /* Project current velocity on velocity obstacle. */
      /* Check if current velocity is projected on cutoff circles. */
      const float t =
          obstacle1 == obstacle2
              ? 0.5F
              : (velocity_ - leftCutoff) * cutoffVector / absSq(cutoffVector);
      const float tLeft = (velocity_ - leftCutoff) * leftLegDirection;
      const float tRight = (velocity_ - rightCutoff) * rightLegDirection;

      if ((t < 0.0F && tLeft < 0.0F) ||
          (obstacle1 == obstacle2 && tLeft < 0.0F && tRight < 0.0F))
      {
        /* Project on left cut-off circle. */
        const Vector2 unitW = normalize(velocity_ - leftCutoff);

        line.direction = Vector2(unitW.y(), -unitW.x());
        line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
        orcaLines_.push_back(line);
        continue;
      }

      if (t > 1.0F && tRight < 0.0F)
      {
        /* Project on right cut-off circle. */
        const Vector2 unitW = normalize(velocity_ - rightCutoff);

        line.direction = Vector2(unitW.y(), -unitW.x());
        line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
        orcaLines_.push_back(line);
        continue;
      }
      /* Project on left leg, right leg, or cut-off line, whichever is closest to
       * velocity. */
      const float distSqCutoff =
          (t < 0.0F || t > 1.0F || obstacle1 == obstacle2)
              ? std::numeric_limits<float>::infinity()
              : absSq(velocity_ - (leftCutoff + t * cutoffVector));
      const float distSqLeft =
          tLeft < 0.0F
              ? std::numeric_limits<float>::infinity()
              : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection));
      const float distSqRight =
          tRight < 0.0F
              ? std::numeric_limits<float>::infinity()
              : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection));

      if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
      {
        /* Project on cut-off line. */
        line.direction = -obstacle1->direction_;
        line.point =
            leftCutoff + radius_ * invTimeHorizonObst *
                             Vector2(-line.direction.y(), line.direction.x());
        orcaLines_.push_back(line);
        continue;
      }
      if (distSqLeft <= distSqRight)
      {
        /* Project on left leg. */
        if (isLeftLegForeign)
        {
          continue;
        }
        line.direction = leftLegDirection;
        line.point =
            leftCutoff + radius_ * invTimeHorizonObst *
                             Vector2(-line.direction.y(), line.direction.x());
        orcaLines_.push_back(line);
        continue;
      }

      /* Project on right leg. */
      if (isRightLegForeign)
      {
        continue;
      }
      line.direction = -rightLegDirection;
      line.point =
          rightCutoff + radius_ * invTimeHorizonObst *
                            Vector2(-line.direction.y(), line.direction.x());
      orcaLines_.push_back(line);
    }
    const std::size_t numObstLines = orcaLines_.size();
    const float invTimeHorizon = 1.0F / timeHorizon_;
    /* Create agent ORCA lines. */
    for (std::size_t i = 0U; i < agentNeighbors_.size(); ++i)
    {
      const RVO::Agent *const other = agentNeighbors_[i];
      const Vector2 relativePosition = other->position_ - position_;
      const Vector2 relativeVelocity = velocity_ - other->velocity_;
      const float distSq = absSq(relativePosition);
      const float combinedRadius = radius_ + other->radius_;
      const float combinedRadiusSq = combinedRadius * combinedRadius;
      Line line;
      Vector2 u;
      if (distSq > combinedRadiusSq)
      {
        /* No collision. */
        const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
        /* Vector from cutoff center to relative velocity. */
        const float wLengthSq = absSq(w);
        std::cout << "1111111wLength: " << wLengthSq << std::endl;
        const float dotProduct = w * relativePosition;
        if (dotProduct < 0.0F &&
            dotProduct * dotProduct > combinedRadiusSq * wLengthSq)
        {
          /* Project on cut-off circle. */
          const float wLength = std::sqrt(wLengthSq);
          const Vector2 unitW = w / wLength;
          line.direction = Vector2(unitW.y(), -unitW.x());
          u = (combinedRadius * invTimeHorizon - wLength) * unitW;
        }
        else
        {
          /* Project on legs. */
          const float leg = std::sqrt(distSq - combinedRadiusSq);
          if (det(relativePosition, w) > 0.0F)
          {
            /* Project on left leg. */
            line.direction = Vector2(relativePosition.x() * leg -
                                         relativePosition.y() * combinedRadius,
                                     relativePosition.x() * combinedRadius +
                                         relativePosition.y() * leg) /
                             distSq;
          }
          else
          {
            /* Project on right leg. */
            line.direction = -Vector2(relativePosition.x() * leg +
                                          relativePosition.y() * combinedRadius,
                                      -relativePosition.x() * combinedRadius +
                                          relativePosition.y() * leg) /
                             distSq;
          }
          u = (relativeVelocity * line.direction) * line.direction -
              relativeVelocity;
        }
      }
      else
      {
        /* Collision. Project on cut-off circle of time timeStep. */
        const float invTimeStep = 1.0F / time;
        /* Vector from cutoff center to relative velocity. */
        const Vector2 w = relativeVelocity - invTimeStep * relativePosition;
        const float wLength = abs(w);
        const Vector2 unitW = w / wLength;
        line.direction = Vector2(unitW.y(), -unitW.x());
        u = (combinedRadius * invTimeStep - wLength) * unitW;
      }
      line.point = velocity_ + 0.5F * u;
      orcaLines_.push_back(line);
    }
    const std::size_t lineFail =
        linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);
    ROS_INFO("111111111Velocity2: x=%.2f, y=%.2f", newVelocity_.x(), newVelocity_.y());
    if (lineFail < orcaLines_.size())
    {
      linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
    }
    return newVelocity_;
  }

  void Agent::update(float timeStep)
  {
    velocity_ = newVelocity_;
    position_ += velocity_ * timeStep;
  }
} /* namespace RVO */
