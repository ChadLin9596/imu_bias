#include <ros/ros.h>
#include <math.h>

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}


// Rotation matrix RY1*RX1*RZ1*RY1*RX1*RZ1 then compute the (1,3)(2,3)(3,3)(2,1)(2,2) will get the result
void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                        float &ox, float &oy, float &oz)
{
  /*R_wl=[ccy 0 scy;0 1 0;-scy 0 ccy]*[1 0 0;0 ccx -scx;0 scx ccx]*[ccz -scz 0;scz ccz 0;0 0 1];（表示以world为参考坐标系）
   *R_cl=[clz -slz 0;slz clz 0;0 0 1]*[1 0 0;0 clx -slx;0 slx clx]*[cly 0 sly;0 1 0;-sly 0 cly];（表示以current为参考坐标系）
   *R_wc=R_wl*(R_cl).';
   *最后求出来(-sin(rx))=cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx) - cos(cx)*cos(lx)*sin(cz)*sin(ly)
   *而进程中是(-sin(rx))= cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);（进程里的srx=(-sin(rx))）
   *可以发现两个公式之间差了lx,ly,lz的负号，所以accumulateRotation()函数传入的是transform[0]~[2]的负值
   *至于为什么-sinx等于上式，可以通过看R_wl，发现第二行第三列的元素为-sinx，因此两个旋转矩阵相乘后，对应位置上的元素就对应着新的pitch角的sin 值
   */
  double srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  double srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy))
                + cos(lx)*sin(ly)*(cos(cy)*cos(cz) + sin(cx)*sin(cy)*sin(cz))
                + cos(lx)*cos(ly)*cos(cx)*sin(cy);

  double crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy)
                - cos(lx)*sin(ly)*(cos(cz)*sin(cy)- cos(cy)*sin(cx)*sin(cz))
                - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));

  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  double srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz))
                + cos(cx)*sin(cz)*(cos(ly)*cos(lz) + sin(lx)*sin(ly)*sin(lz))
                + cos(lx)*cos(cx)*cos(cz)*sin(lz);

  double crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz)
                - cos(cx)*sin(cz)*(cos(ly)*sin(lz) - cos(lz)*sin(lx)*sin(ly))
                - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));

  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}
void AccumulateRotation_2(float rx, float ry, float rz, float ox, float oy, float oz,
                          float &lx, float &ly, float &lz){
double crx = cos(rx);double srx = sin(rx);
double cry = cos(ry);double sry = sin(ry);
double crz = cos(rz);double srz = sin(rz);

double cox = cos(ox);double sox = sin(ox);
double coy = cos(oy);double soy = sin(oy);
double coz = cos(oz);double soz = sin(oz);

double slx = cox*crx*soz*sry - crx*crz*sox - srx*cox*coy;

lx = asin(-slx);

double clxsly = (cry*crz + srx*sry*srz)*cox*soy
              + (-cry*srz + srx*sry*crz)*(-sox)
              + crx*sry*cox*coy;
double clxcly = (-sry*crz+srx*cry*srz)*cox*soy
              + (sry*srz+srx*cry*crz)*(-sox)
              + crx*cry*cox*coy;

ly = atan2(clxsly/cos(lx), clxcly/cos(lx));

double clxslz = crx*srz*(coy*coz+sox*soy*soz)
              + crx*crz*cox*soz
              + (-srx)*(-soy*coz+sox*coy*soz);

double clxcoz = crx*srz*(-coy*soz+sox*soy*coz)
              + crx*crz*cox* coz
              + (-srx)*(soy*soz+sox*coy*coz);

lz = atan2(clxslz/cos(lx),clxcoz/cos(lx));
}

float rx1 = 0, ry1 = 0 , rz1 = 0;
float rx2 = 0, ry2 = 0, rz2 = 0;
float ox = 0, oy = 0, oz = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ncrl_test");
  ros::NodeHandle nh;

  rx1 = deg2rad(10);
  ry1 = deg2rad(10);
  rz1 = deg2rad(0);

  rx2 = deg2rad(25);
  ry2 = deg2rad(35);
  rz2 = deg2rad(0);

  AccumulateRotation(rx1,ry1,rz1,rx2,ry2,rz2,ox,oy,oz);
  printf("x1 : %f y1 : %f z1 : %f\n",rad2deg(ox),rad2deg(oy),rad2deg(oz));

  AccumulateRotation_2(rx1,ry1,rz1,rx2,ry2,rz2,ox,oy,oz);
  printf("x2 : %f y2 : %f z2 : %f\n",rad2deg(ox),rad2deg(oy),rad2deg(oz));
}
