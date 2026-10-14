// 初学者导读（underwater_devices / test/attitude_reference_test.cpp）
// 所属：姿态参考的单元测试，直接链接纯 C++ 实现，不需要 ROS。
// 机制：main() 里顺序执行断言，任何一条失败就抛异常并打印 FAIL；全部通过打印 PASS。
// 测试内容：四元数/RPY 换算与边界、静止采集状态机、标定文件读写与原子替换。
// C++ 概念：匿名命名空间、模板函数 rejects() 接收 lambda 作为“被测动作”、
//   check()/near() 是自定义断言辅助；mkdtemp 创建临时目录存放测试文件。
#include "underwater_devices/attitude_reference.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using namespace underwater::devices;
namespace fs=std::filesystem;
// 匿名命名空间：以下辅助函数与计数器仅本文件可见。
namespace {
// checks 记录已执行的断言数量，用于最终 PASS 统计。
int checks=0;
constexpr double rad=3.14159265358979323846/180;
// check()：断言 ok 为真，否则抛出带原因的异常。
void check(bool ok,const std::string &why) { ++checks; if(!ok) throw std::runtime_error(why); }
// near()：判断两数之差小于 1e-8（浮点数比较不能用 ==）。
void near(double a,double b,const std::string &why) { check(std::abs(a-b)<1e-8,why); }
// rejects()：断言 f() 会抛异常，用于测试非法输入被拒绝。
template<class F> void rejects(F f,const std::string &why) {
  bool threw=false;try { f(); } catch(const std::exception &) { threw=true; } check(threw,why);
}
// pose()：模拟一次“原始角速度 + 姿态”输入，便于连续喂样本。
void pose(ReferenceCapture &c,Vector3 rpy,double time,std::uint64_t connection=4) {
  c.raw({0,0,0},true,connection,time); c.attitude(rpy,true,connection,"imu_sensor",time);
}
}
// main()：测试入口，返回 0 表示全部通过。
int main() {
  // 在 /tmp 下创建唯一临时目录，测试文件都放在里面。
  char directory[]="/tmp/underwater-imu-reference-test-XXXXXX";
  if(!mkdtemp(directory)) return 2;
  const fs::path root(directory);
  // 主测试体放在 try 中，任何异常都会走 catch 打印 FAIL。
  try {
    // 第一组：相对姿态换算的正确性（已知姿态、目标角、旋转叠加、yaw 回绕）。
    AttitudeReference ref; const Vector3 baseline{.2,-.3,2.9}; ref.baseline=rpy_quaternion(baseline);
    auto output=quaternion_rpy(reference_attitude(ref,baseline));
    for(double v:output) near(v,0,"known pose becomes zero");
    ref.target_rpy_deg={10,-20,90}; output=quaternion_rpy(reference_attitude(ref,baseline));
    for(std::size_t i=0;i<3;++i) near(output[i],ref.target_rpy_deg[i]*rad,"known pose becomes target");
    const auto delta=rpy_quaternion({.3,.15,-.2});
    const auto measured=quaternion_rpy(quaternion_product(ref.baseline,delta));
    const auto expected=quaternion_product(rpy_quaternion({10*rad,-20*rad,90*rad}),delta);
    check(quaternion_distance(reference_attitude(ref,measured),expected)<1e-7,"noncommuting compound rotations");
    ref.target_rpy_deg={};ref.baseline=rpy_quaternion({0,0,179*rad});
    near(quaternion_rpy(reference_attitude(ref,{0,0,-179*rad}))[2],2*rad,"yaw wrap is two degrees");
    for(double pitch:{-90.0,-89.999,89.999,90.0}) {
      const auto q=rpy_quaternion({.1,pitch*rad,.3});
      const auto rpy=quaternion_rpy(q);for(double v:rpy) check(std::isfinite(v),"pitch boundary finite");
    }
    rejects([&]{rpy_quaternion({0,std::numeric_limits<double>::quiet_NaN(),0});},"NaN rejected");
    rejects([&]{normalized_quaternion({0,0,0,0});},"zero quaternion rejected");
    // 第二组：ReferenceCapture 状态机的各种拒绝/完成路径。
    ReferenceCapture c;
    rejects([&]{c.start({0,91,0},10,5,0);},"invalid target rejected");
    rejects([&]{c.start({},9,5,0);},"too few samples rejected");
    c.start({},10,5,0);
    rejects([&]{c.start({},10,5,0);},"concurrent capture rejected");
    c.attitude({},true,4,"imu_sensor",.01);check(c.count()==0,"gyro required after request");
    for(int i=0;i<100;++i) pose(c,baseline,.1);
    check(c.count()==1 && c.busy(),"buffered burst does not pass stationary duration");
    for(int i=1;i<10;++i) pose(c,baseline,.1+i*.12);
    auto candidate=c.take_candidate();check(candidate.has_value() && !c.busy(),"stationary capture completes");
    check(candidate->samples==10,"count recorded");
    check(quaternion_distance(candidate->baseline,rpy_quaternion(baseline))<1e-7,"mean baseline accurate");
    c.start({},10,5,10);pose(c,{},10.1);c.raw({0,0,.1},true,4,10.2);
    check(!c.busy() && !c.take_candidate(),"gyro motion rejects entire capture");
    c.start({},10,5,20);pose(c,{},20.1);pose(c,{0,0,2*rad},20.3);
    check(!c.busy(),"attitude spread rejects capture");
    c.start({},10,5,30);pose(c,{},30.1);pose(c,{},30.3,5);
    check(!c.busy(),"reconnect rejects capture");
    c.start({},10,5,40);pose(c,{},40.1);c.raw({},false,4,40.2);
    check(!c.busy(),"invalid raw sample rejects capture");
    c.start({},10,5,50);c.tick(55);check(!c.busy(),"no-data timeout");
    c.start({},10,5,60);pose(c,{},60.1);c.attitude({},true,4,"imu_sensor",61);
    check(c.count()==1,"stale gyro cannot advance capture");c.cancel("cancelled");
    CaptureLimits limits;limits.spread_limit_deg=5;
    ReferenceCapture wrap(limits);wrap.start({},10,5,0);
    for(int i=0;i<10;++i) pose(wrap,{0,0,(i%2? -179.9:179.9)*rad},i*.12);
    auto average=wrap.take_candidate();check(average.has_value(),"wraparound samples accepted");
    check(quaternion_distance(average->baseline,rpy_quaternion({0,0,180*rad}))<1e-7,"hemisphere average handles yaw wrap");
    // 第三组：标定文件的保存、读取、原子替换与损坏文件拒绝。
    const auto file=(root/"nested"/"reference.txt").string();
    check(!load_attitude_reference(file,"imu_sensor"),"missing file is uncalibrated");
    candidate->saved_at="2026-10-08T00:00:00Z";
    save_attitude_reference(file,*candidate);
    auto loaded=load_attitude_reference(file,"imu_sensor");
    check(loaded && loaded->samples==10 && loaded->saved_at==candidate->saved_at,"persistent metadata round trip");
    check(quaternion_distance(loaded->baseline,candidate->baseline)<1e-7,"persistent precision");
    rejects([&]{load_attitude_reference(file,"another_imu");},"frame mismatch rejected");
    auto bad=*candidate;bad.target_rpy_deg[0]=999;
    rejects([&]{save_attitude_reference(file,bad);},"invalid overwrite rejected");
    check(load_attitude_reference(file,"imu_sensor")->target_rpy_deg==candidate->target_rpy_deg,"previous file survives save rejection");
    candidate->target_rpy_deg={0,0,90};save_attitude_reference(file,*candidate);
    near(load_attitude_reference(file,"imu_sensor")->target_rpy_deg[2],90,"atomic replacement");
    const auto link=(root/"linked.txt").string();fs::create_symlink(file,link);
    rejects([&]{clear_attitude_reference(link);},"do not clear symlink target");
    check(fs::exists(file),"symlink target retained");
    {std::ofstream out(file);out<<"invalid calibration";}
    rejects([&]{load_attitude_reference(file,"imu_sensor");},"corrupt file rejected");
    clear_attitude_reference(file);check(!fs::exists(file),"clear removes active file");
    clear_attitude_reference(file);check(!load_attitude_reference(file,"imu_sensor"),"clear is idempotent");
    // 清理临时目录并报告结果。
    fs::remove_all(root);
    std::cout<<"PASS "<<checks<<" IMU attitude reference checks\n";return 0;
  } catch(const std::exception &e) {
    std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<" (evidence: "<<root<<")\n";return 1;
  }
}
