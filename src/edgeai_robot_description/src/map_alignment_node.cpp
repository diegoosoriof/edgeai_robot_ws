#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;
struct Pose {
  double x{0}, y{0}, a{0};
};
double wrap(double a) {
  while (a >= kPi) {
    a -= 2 * kPi;
  }
  while (a < -kPi) {
    a += 2 * kPi;
  }
  return a;
}
double angleDiff(double a, double b) {
  return wrap(a - b);
}
struct Point {
  double x, y;
};

struct MapData {
  int w{0}, h{0};
  double res{0}, ox{0}, oy{0}, oyaw{0};
  cv::Mat distance, known, flann_points;
  std::vector<Point> points;
  std::unique_ptr<cv::flann::Index> tree;
  Point gridWorld(double gx, double gy) const {
    double lx = (gx + 0.5) * res, ly = (gy + 0.5) * res;
    return {ox + std::cos(oyaw) * lx - std::sin(oyaw) * ly, oy + std::sin(oyaw) * lx + std::cos(oyaw) * ly};
  }
  bool lookup(double x, double y, double &d) const {
    double dx = x - ox, dy = y - oy;
    double lx = std::cos(oyaw) * dx + std::sin(oyaw) * dy;
    double ly = -std::sin(oyaw) * dx + std::cos(oyaw) * dy;
    int gx = static_cast<int>(std::floor(lx / res)), gy = static_cast<int>(std::floor(ly / res));
    if(gx < 0||gy < 0||gx >= w||gy >= h)
      return false;
    int row = h - 1 - gy;
    if(!known.at<unsigned char>(row,gx))
      return false;
    d = distance.at<float>(row,gx);
    return true;
  }
};
std::shared_ptr<const MapData> preprocess(const nav_msgs::msg::OccupancyGrid &m, int free_t, int occ_t, int sample) {
  const auto w = static_cast<int>(m.info.width), h = static_cast<int>(m.info.height);
  if (w <= 0 || h <= 0 || m.info.resolution <= 0 ||
      !std::isfinite(m.info.resolution) ||
      m.data.size() != static_cast<size_t>(w) * h) {
    return {};
  }
  auto d = std::make_shared<MapData>();
  d->w = w;
  d->h = h;
  d->res = m.info.resolution;
  d->ox = m.info.origin.position.x;
  d->oy = m.info.origin.position.y;
  const auto &q = m.info.origin.orientation;
  d->oyaw = std::atan2(2 * (q.w * q.z + q.x * q.y),1 - 2 * (q.y * q.y + q.z * q.z));
  cv::Mat occ(h,w,CV_8UC1,cv::Scalar(0));
  d->known = cv::Mat(h,w,CV_8UC1,cv::Scalar(0));
  for(int gy = 0; gy < h; ++gy)
    for(int x = 0; x < w; ++x)
    {
      int v = m.data[static_cast<size_t>(gy) * w + x], row = h - 1 - gy;
      if(v >= 0&&(v <= free_t||v >= occ_t))
        d->known.at<unsigned char>(row,x) = 255;
      if(v >= occ_t)
      {
        occ.at<unsigned char>(row,x) = 255;
        if(gy % sample == 0&&x % sample == 0)
          d->points.push_back(d->gridWorld(x,gy));
      }
    }
  if(d->points.empty())
    return {};
  cv::Mat inv;
  cv::compare(occ,0,inv,cv::CMP_EQ);
  cv::distanceTransform(inv,d->distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  d->distance *= static_cast<float>(d->res);
  d->flann_points = cv::Mat(static_cast<int>(d->points.size()),2,CV_32F);
  for(size_t i = 0; i < d->points.size(); ++i)
  {
    d->flann_points.at<float>(static_cast<int>(i),0) = d->points[i].x;
    d->flann_points.at<float>(static_cast<int>(i),1) = d->points[i].y;
  }
  d->tree = std::make_unique<cv::flann::Index>(d->flann_points,cv::flann::KDTreeIndexParams(4));
  return d;
}
struct Result {
  Pose pose;
  double score{1e9};
  size_t overlap{0};
  double inlier{0};
};
struct Candidate {
  Pose p;
  double score;
  size_t overlap;
};
struct Params {
  double radius{5}, xy{.25}, theta{4}, nmsd{.5}, nmsa{15};
  double maxcorr{.5}, icpcorr{.5}, eps{1e-4}, accept{.125}, ambig{.7};
  double stablexy{.1}, stablea{2};
  int stride{4}, topk{10}, mino{30}, iters{50}, stablecount{3}, sampling{2}, threads{1};
};
Point transform(const Pose&p,Point q) {
  return {p.x + std::cos(p.a) * q.x - std::sin(p.a) * q.y,p.y + std::sin(p.a) * q.x + std::cos(p.a) * q.y};
}
Result scorePose(const MapData&a,const MapData&b,const Pose&p,const Params&z,int stride) {
  double sum = 0;
  size_t n = 0;
  const double cap = z.maxcorr * z.maxcorr;
  auto eval = [&](const MapData &src,const MapData &dst,const Pose &t) {
      for(size_t i = 0; i < src.points.size(); i += static_cast<size_t>(std::max(1,stride)))
      {
        Point q = transform(t,src.points[i]);
        double dist;
        if(dst.lookup(q.x,q.y,dist))
        {
          sum += std::min(cap,dist * dist);
          ++n;
        }
      }
    };
  eval(b,a,p);
  Pose inv{-std::cos(p.a) * p.x - std::sin(p.a) * p.y,std::sin(p.a) * p.x - std::cos(p.a) * p.y,-p.a};
  eval(a,b,inv);
  Result r;
  r.pose = p;
  r.overlap = n;
  r.score = n < static_cast<size_t>(z.mino)?cap:sum / n;
  size_t close = 0, considered = 0;
  for(const auto &q:b.points)
  {
    Point t = transform(p,q);
    double d;
    if(a.lookup(t.x,t.y,d))
    {
      ++considered;
      if(d <= 2 * a.res)
        ++close;
    }
  }
  r.inlier = considered?static_cast<double>(close) / considered:0;
  return r;
}
}

class MapAlignmentNode : public rclcpp::Node {
public:
  MapAlignmentNode() : Node("map_alignment"),tf_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)) {
    p_.radius = param("search_radius",5.0);
    p_.xy = param("coarse_xy_step",.25);
    p_.theta = param("coarse_theta_step_deg",4.0);
    p_.stride = iparam("coarse_point_stride",4);
    p_.topk = iparam("top_k",10);
    p_.nmsd = param("nms_distance",.5);
    p_.nmsa = param("nms_angle_deg",15.0);
    p_.maxcorr = param("max_correspondence_distance",.5);
    p_.icpcorr = param("icp_max_correspondence_distance",.5);
    p_.iters = iparam("icp_max_iterations",50);
    p_.eps = param("icp_epsilon",1e-4);
    p_.mino = iparam("min_overlap_points",30);
    p_.accept = param("accept_score",.5 * p_.maxcorr * p_.maxcorr);
    p_.ambig = param("ambiguity_ratio_threshold",.7);
    p_.stablecount = iparam("stable_count",3);
    p_.stablexy = param("stable_translation_tol",.1);
    p_.stablea = param("stable_rotation_tol_deg",2.0);
    free_ = iparam("free_threshold",20);
    occ_ = iparam("occupied_threshold",50);
    p_.sampling = iparam("occupied_point_sampling_step",2);
    int hw = static_cast<int>(std::thread::hardware_concurrency());
    p_.threads = iparam("num_threads",std::max(1,hw));
    map1_ = declare_parameter<std::string>("map1_topic","/robot1/map");
    map2_ = declare_parameter<std::string>("map2_topic","/robot2/map");
    source_ = declare_parameter<std::string>("source_frame","robot2/map");
    target_ = declare_parameter<std::string>("target_frame","robot1/map");
    publish_tf_ = declare_parameter<bool>("publish_tf",true);
    markers_ = declare_parameter<bool>("enable_markers",false);
    (void)declare_parameter<bool>("self_test",false);
    clamp();
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    sub1_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map1_,qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      receive(m,0);
    });
    sub2_ = create_subscription<nav_msgs::msg::OccupancyGrid>(map2_,qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m){
      receive(m,1);
    });
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("alignment",10);
    transform_pub_ = create_publisher<geometry_msgs::msg::TransformStamped>("alignment_transform",10);
    status_pub_ = create_publisher<std_msgs::msg::String>("alignment_status",10);
    if(markers_)
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("alignment_markers",10);
    reset_srv_ = create_service<std_srvs::srv::Trigger>("~/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      {std::lock_guard<std::mutex> l(m_);
       latched_ = false;
       stable_ = 0;
       ++v1_;
      }
      cv_.notify_one();
      response->success = true;
      response->message = "Alignment latch cleared; search restarted";
    });
    worker_ = std::thread([this]{
      work();
    });
  }
  ~MapAlignmentNode() override {
    {
      std::lock_guard<std::mutex> l(m_);
      stopping_ = true;
    }
    cv_.notify_one();
    if(worker_.joinable())
      worker_.join();
  }
private:
  double param(const std::string&n,double v){
    return declare_parameter<double>(n,v);
  }
  int iparam(const std::string&n,int v){
    return declare_parameter<int>(n,v);
  }
  void clamp(){
    p_.radius = std::max(.01,p_.radius);
    p_.xy = std::max(.01,p_.xy);
    p_.theta = std::max(.1,p_.theta);
    p_.stride = std::max(1,p_.stride);
    p_.topk = std::max(1,p_.topk);
    p_.maxcorr = std::max(.01,p_.maxcorr);
    p_.icpcorr = std::max(.01,p_.icpcorr);
    p_.iters = std::max(1,p_.iters);
    p_.eps = std::max(1e-8,p_.eps);
    p_.mino = std::max(1,p_.mino);
    p_.threads = std::max(1,p_.threads);
    p_.sampling = std::max(1,p_.sampling);
  }
  void receive(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg,int which){
    auto d = preprocess(*msg,free_,occ_,p_.sampling);
    if(!d)
    {
      RCLCPP_WARN(get_logger(),"Ignoring invalid map on %s",which?map2_.c_str():map1_.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> l(m_);
      if(which)
      {
        b_ = std::move(d);
        ++v2_;
      }
      else{
        a_ = std::move(d);
        ++v1_;
      }
    }
    cv_.notify_one();
  }
  std::vector<Candidate> search(const MapData&a,const MapData&b,size_t&count){
    std::vector<double> angles;
    double astep = p_.theta * kPi / 180;
    int na = std::max(1,static_cast<int>(std::ceil(2 * kPi / astep)));
    for(int i = 0; i < na; ++i)
      angles.push_back(i * 2 * kPi / na);
    std::vector<Point> xy;
    int n = static_cast<int>(std::ceil(p_.radius / p_.xy));
    for(int iy = -n; iy <= n; ++iy)
      for(int ix = -n; ix <= n; ++ix)
      {
        double x = ix * p_.xy,y = iy * p_.xy;
        if(x * x + y * y <= p_.radius * p_.radius)
          xy.push_back({x,y});
      }
    std::vector<Pose> poses;
    poses.reserve(xy.size() * angles.size());
    for(auto q:xy)
      for(double t:angles)
        poses.push_back({q.x,q.y,t});
    std::vector<Candidate> results(poses.size());
    std::atomic<size_t> next{0};
    std::vector<std::thread> pool;
    for(int k = 0; k < std::min<int>(p_.threads,poses.size()); ++k)
      pool.emplace_back([&]{
        for(;;)
        {
          size_t i = next.fetch_add(1);
          if(i >= poses.size())
            break;
          auto r = scorePose(a,b,poses[i],p_,p_.stride);
          results[i] = {r.pose,r.score,r.overlap};
        }
      });
    for(auto&t:pool)
      t.join();
    count = poses.size();
    std::sort(results.begin(),results.end(),[](auto&x,auto&y){
      return x.score < y.score;
    });
    std::vector<Candidate> top;
    for(auto c:results)
    {
      bool near = false;
      for(auto&q:top)
        if(std::hypot(c.p.x - q.p.x,c.p.y - q.p.y) < p_.nmsd&&std::abs(angleDiff(c.p.a,q.p.a)) * 180 / kPi < p_.nmsa)
        {
          near = true;
          break;
        }
      if(!near)
        top.push_back(c);
      if(top.size() >= static_cast<size_t>(p_.topk))
        break;
    }
    std::vector<Candidate> refined;
    double dx = p_.xy / 4, da = astep / 4;
    for(auto seed:top)
    {
      Candidate best = seed;
      for(double x = seed.p.x - p_.xy; x <= seed.p.x + p_.xy + dx / 2; x += dx)
        for(double y = seed.p.y - p_.xy; y <= seed.p.y + p_.xy + dx / 2; y += dx)
          for(int it = -4; it <= 4; ++it)
          {
            Pose q{x,y,wrap(seed.p.a + it * da)};
            auto r = scorePose(a,b,q,p_,1);
            ++count;
            if(q.x * q.x + q.y * q.y > p_.radius * p_.radius)
              continue;
            if(r.score < best.score)
              best = {q,r.score,r.overlap};
          }
      refined.push_back(best);
    }
    return refined;
  }
  Result icp(const MapData&a,const MapData&b,Candidate c){
    Pose p = c.p;
    for(int iter = 0; iter < p_.iters; ++iter)
    {
      std::vector<Point> x,y;
      cv::Mat query(1,2,CV_32F),indices(1,1,CV_32S),dists(1,1,CV_32F);
      for(auto q:b.points)
      {
        Point t = transform(p,q);
        query.at<float>(0,0) = t.x;
        query.at<float>(0,1) = t.y;
        a.tree->knnSearch(query,indices,dists,1,cv::flann::SearchParams(32));
        if(std::sqrt(dists.at<float>(0,0)) <= p_.icpcorr)
        {
          int j = indices.at<int>(0,0);
          x.push_back(q);
          y.push_back(a.points[static_cast<size_t>(j)]);
        }
      }
      if(x.size() < 3)
        break;
      Point mx{0,0},my{0,0};
      for(size_t i = 0; i < x.size(); ++i)
      {
        mx.x += x[i].x;
        mx.y += x[i].y;
        my.x += y[i].x;
        my.y += y[i].y;
      }
      mx.x /= x.size();
      mx.y /= x.size();
      my.x /= y.size();
      my.y /= y.size();
      double cs = 0,sn = 0;
      for(size_t i = 0; i < x.size(); ++i)
      {
        double ax = x[i].x - mx.x,ay = x[i].y - mx.y,bx = y[i].x - my.x,by = y[i].y - my.y;
        cs += ax * bx + ay * by;
        sn += ax * by - ay * bx;
      }
      double da = std::atan2(sn,cs);

      Pose delta{
        my.x - (std::cos(da) * mx.x - std::sin(da) * mx.y),
        my.y - (std::sin(da) * mx.x + std::cos(da) * mx.y), da};

      Pose np{delta.x, delta.y, wrap(da)};

      double change = std::hypot(np.x - p.x,np.y - p.y) + std::abs(angleDiff(np.a,p.a));
      p = np;
      if(change < p_.eps)
        break;
    }
    auto r = scorePose(a,b,p,p_,1);
    return r;
  }
  geometry_msgs::msg::TransformStamped makeTf(const Pose&p){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = target_;
    t.child_frame_id = source_;
    t.transform.translation.x = p.x;
    t.transform.translation.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0,0,wrap(p.a));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    return t;
  }
  void publish(const Result&r,double second,double ratio,size_t candidates,double ms,bool conv,bool latched){
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = now();
    m.header.frame_id = target_;
    m.pose.pose.position.x = r.pose.x;
    m.pose.pose.position.y = r.pose.y;
    tf2::Quaternion q;
    q.setRPY(0,0,wrap(r.pose.a));
    m.pose.pose.orientation.x = q.x();
    m.pose.pose.orientation.y = q.y();
    m.pose.pose.orientation.z = q.z();
    m.pose.pose.orientation.w = q.w();
    double cov = conv?.01:1e3;
    m.pose.covariance[0] = cov;
    m.pose.covariance[7] = cov;
    m.pose.covariance[35] = conv?.01:1e3;
    pose_pub_->publish(m);
    if(conv)
    {
      auto t = makeTf(r.pose);
      transform_pub_->publish(t);
      if(publish_tf_)
        tf_->sendTransform(t);
    }
    std::ostringstream s;
    s << "best_score=" << r.score
      << " second_best_score=" << second
      << " ambiguity_ratio=" << ratio
      << " overlap_points=" << r.overlap
      << " inlier_ratio=" << r.inlier
      << " candidates_evaluated=" << candidates
      << " search_time_ms=" << ms
      << " converged=" << (conv ? "true" : "false")
      << " latched=" << (latched ? "true" : "false");
    std_msgs::msg::String st;
    st.data = s.str();
    status_pub_->publish(st);
  }
  void work(){
    uint64_t done1 = 0,done2 = 0;
    while(true){
      std::shared_ptr<const MapData>a,b;
      uint64_t x,y;
      {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l,[&]{
          return stopping_||(a_&&b_&&!latched_&&(v1_ != done1||v2_ != done2));
        });
        if(stopping_)
          return;
        a = a_;
        b = b_;
        x = v1_;
        y = v2_;
      }
      auto start = std::chrono::steady_clock::now();
      size_t count = 0;
      auto c = search(*a,*b,count);
      std::vector<Result> finals;
      for(auto q:c)
        finals.push_back(icp(*a,*b,q));
      std::sort(finals.begin(),finals.end(),[](auto&u,auto&v){
        return u.score < v.score;
      });
      if(finals.empty())
        continue;
      Result best = finals[0];
      double second = finals.size() > 1?finals[1].score:best.score;
      double ratio = second > 1e-12?best.score / second:0;
      bool converged = best.score < p_.accept&&best.overlap >= static_cast<size_t>(p_.mino)&&ratio < p_.ambig;
      bool latched = false;
      {
        std::lock_guard<std::mutex> l(m_);
        done1 = x;
        done2 = y;
        if(converged)
        {
          if (stable_ == 0 ||
              std::hypot(best.pose.x, last_.x) > p_.stablexy ||
              std::abs(angleDiff(best.pose.a, last_.a)) * 180 / kPi > p_.stablea) {
            stable_ = 1;
          } else {
            ++stable_;
          }
          last_ = best.pose;
          if(stable_ >= p_.stablecount)
          {
            latched_ = true;
            latched = true;
          }
        }
        else stable_ = 0;
      }
      double ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - start).count();
      publish(best,second,ratio,count,ms,converged,latched);
      RCLCPP_INFO(
        get_logger(),
        "Alignment score %.5f, overlap %zu, ambiguity %.3f, "
        "%zu candidates in %.1f ms%s",
        best.score, best.overlap, ratio, count, ms,
        latched ? "; latched" : "");
    }
  }
  int free_{20},occ_{50};
  std::string map1_,map2_,source_,target_;
  bool publish_tf_{true},markers_{false},stopping_{false},latched_{false};
  Params p_;
  int stable_{0};
  Pose last_{};
  uint64_t v1_{0},v2_{0};
  std::shared_ptr<const MapData>a_,b_;
  std::mutex m_;
  std::condition_variable cv_;
  std::thread worker_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub1_,sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>tf_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MapAlignmentNode>());
  rclcpp::shutdown();
  return 0;
}
