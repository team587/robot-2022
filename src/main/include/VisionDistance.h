#pragma once

class VisionDistance {
    public:
    VisionDistance() {};
    VisionDistance(double minDistance, double maxDistance, double hoodAngle,  double shooterSpeed){
      m_minDistance = minDistance;
      m_maxDistance = maxDistance;
      m_hoodAngle = hoodAngle;
      m_shooterSpeed = shooterSpeed;
    };
    double m_minDistance;
    double m_maxDistance;
    double m_hoodAngle;
    double m_shooterSpeed;
    bool isDistance(double distance){
      if(distance >= m_minDistance && distance < m_maxDistance){
         return true;
      }
      return false;
    }
  };