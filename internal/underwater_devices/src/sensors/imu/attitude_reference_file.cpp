#include "underwater_devices/attitude_reference.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace underwater::devices {
namespace fs=std::filesystem;
namespace {
void validate(const AttitudeReference &r) {
  (void)reference_attitude(r,{0,0,0});
  double norm=0; for (double v:r.baseline) norm+=v*v;
  if (std::abs(norm-1)>1e-6 || r.samples<10 || r.samples>2000 || r.frame_id.empty() ||
      r.frame_id.size()>128 || r.saved_at.empty() || r.saved_at.size()>64)
    throw std::runtime_error("invalid calibration file fields");
}
void regular_or_absent(const fs::path &p) {
  const auto status=fs::symlink_status(p);
  if (fs::exists(status) && !fs::is_regular_file(status))
    throw std::runtime_error("calibration path must be a regular file, not a directory or symlink");
}
struct FileLock {
  int fd=-1;
  explicit FileLock(const std::string &path) {
    fd=::open((path+".lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if (fd<0) throw std::runtime_error("cannot open calibration lock: "+std::string(std::strerror(errno)));
    if (::flock(fd,LOCK_EX|LOCK_NB)!=0) { ::close(fd); fd=-1; throw std::runtime_error("calibration file is busy"); }
  }
  ~FileLock() { if(fd>=0) ::close(fd); }
};
}
std::optional<AttitudeReference> load_attitude_reference(const std::string &path,const std::string &frame) {
  regular_or_absent(path);
  if (!fs::exists(path)) return std::nullopt;
  if (fs::file_size(path)>4096) throw std::runtime_error("calibration file is too large");
  std::ifstream in(path); if(!in) throw std::runtime_error("cannot read calibration file");
  std::string key,source; int version=0; AttitudeReference r;
  if (!(in>>key>>version) || key!="UNDERWATER_IMU_REFERENCE" || version!=1)
    throw std::runtime_error("unsupported calibration file version");
  if (!(in>>key>>std::quoted(source)) || key!="source" || source!="euler_zyx_relative")
    throw std::runtime_error("unsupported calibration orientation convention");
  if (!(in>>key>>std::quoted(r.frame_id)) || key!="frame") throw std::runtime_error("missing calibration frame");
  if (!(in>>key>>r.baseline[0]>>r.baseline[1]>>r.baseline[2]>>r.baseline[3]) || key!="baseline_wxyz")
    throw std::runtime_error("invalid calibration baseline");
  if (!(in>>key>>r.target_rpy_deg[0]>>r.target_rpy_deg[1]>>r.target_rpy_deg[2]) || key!="target_rpy_deg")
    throw std::runtime_error("invalid calibration target");
  if (!(in>>key>>r.samples) || key!="samples") throw std::runtime_error("invalid calibration sample count");
  if (!(in>>key>>std::quoted(r.saved_at)) || key!="saved_at") throw std::runtime_error("missing calibration date");
  if (in>>key) throw std::runtime_error("unexpected trailing calibration data");
  validate(r);
  if(r.frame_id!=frame) throw std::runtime_error("saved calibration frame differs from imu_frame_id");
  return r;
}
void save_attitude_reference(const std::string &path,const AttitudeReference &r) {
  validate(r); const fs::path p(path);
  if(!p.is_absolute()) throw std::invalid_argument("calibration path must be absolute");
  fs::create_directories(p.parent_path()); FileLock lock(path); regular_or_absent(p);
  std::ostringstream out; out<<std::setprecision(17);
  out<<"UNDERWATER_IMU_REFERENCE 1\nsource \"euler_zyx_relative\"\nframe "<<std::quoted(r.frame_id)
     <<"\nbaseline_wxyz";
  for(double v:r.baseline) out<<' '<<v;
  out<<"\ntarget_rpy_deg"; for(double v:r.target_rpy_deg) out<<' '<<v;
  out<<"\nsamples "<<r.samples<<"\nsaved_at "<<std::quoted(r.saved_at)<<'\n';
  std::string temp=path+".tmp.XXXXXX";
  int fd=::mkstemp(temp.data()); if(fd<0) throw std::runtime_error("cannot create calibration temporary file");
  try {
    const auto data=out.str(); std::size_t offset=0;
    while(offset<data.size()) {
      const auto n=::write(fd,data.data()+offset,data.size()-offset);
      if(n<0 && errno==EINTR) continue;
      if(n<=0) throw std::runtime_error("calibration write failed");
      offset+=static_cast<std::size_t>(n);
    }
    if(::fsync(fd)!=0) throw std::runtime_error("calibration flush failed");
    const int result=::close(fd); fd=-1;
    if(result!=0) throw std::runtime_error("calibration close failed");
    // Same-directory POSIX rename: failure leaves the previous active file untouched.
    fs::rename(temp,p);
  } catch(...) { if(fd>=0) ::close(fd); ::unlink(temp.c_str()); throw; }
}
void clear_attitude_reference(const std::string &path) {
  const fs::path p(path);
  if(!p.is_absolute()) throw std::invalid_argument("calibration path must be absolute");
  if(!fs::exists(p.parent_path())) return;
  FileLock lock(path); regular_or_absent(p); fs::remove(p);
}
}  // namespace underwater::devices
