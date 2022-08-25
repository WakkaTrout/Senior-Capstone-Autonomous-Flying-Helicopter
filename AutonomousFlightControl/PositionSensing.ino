inline void GetPositionFromQuaternion(float &pitch, float &roll, float &yaw, float qw, float qx, float qy, float qz)
{
      //convert the quaternion to yaw, pitch, and roll in degrees
      double qysqr = qy * qy;
      double t0 = -2.0 * (qysqr + qz *qz) + 1.0;
      double t1 = +2.0 * (qx * qy + qw * qz);
      double t2 = -2.0 * (qx * qz - qw * qy);
      double t3 = +2.0 * (qy * qz + qw * qx);
      double t4 = -2.0 * (qx * qx + qysqr) + 1.0;

      //clip if greater than or less than +/- 1
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;

      pitch = asin(t2);
      roll = atan2(t3, t4);
      yaw = atan2(t1, t0);
      
      //convert from radians to degrees
      yaw = (yaw *180.0)/ M_PI;
      pitch = (pitch *180.0)/ M_PI;
      roll = (roll *180.0)/ M_PI;
}
