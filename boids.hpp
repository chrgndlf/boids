#ifndef PF_BOIDS_HPP
#define PF_BOIDS_HPP

#include <stdexcept>
#include <vector>

namespace pf {

namespace costants {
inline constexpr double pi = 3.14159265358979323846;
}

struct Vector {
  double x;
  double y;

  Vector& operator+=(Vector const&);
  Vector& operator-=(Vector const&);
  Vector& operator*=(double const);
  Vector& operator/=(double const);
};

Vector operator+(Vector const&, Vector const&);
Vector operator-(Vector const&, Vector const&);
Vector operator*(Vector const&, double const);
Vector operator*(double const, Vector const&);
Vector operator/(Vector const&, double const);
bool operator==(Vector const&, Vector const&);
bool operator!=(Vector const&, Vector const&);

double magnitude(Vector const&);

Vector normalize(Vector const&);

struct Boid {
  Vector p;
  Vector v;
  std::vector<Boid const*> neighbors;
};

class Flock {
 private:
  std::vector<Boid> boids_;
  double d_;
  double ds_;
  double s_;
  double a_;
  double c_;
  Vector v1_(Boid const&) const;
  Vector v2_(Boid const&) const;
  Vector v3_(Boid const&) const;
  static constexpr int width_ = 800;
  static constexpr int height_ = 600;
  static constexpr double min_speed_ = 2.;
  static constexpr double max_speed_ = 5.;

 public:
  explicit Flock(std::vector<Boid> boids, double d, double ds, double s,
                 double a, double c)
      : boids_{boids}, d_{d}, ds_{ds}, s_{s}, a_{a}, c_{c} {
    if (boids_.empty()) {
      throw std::runtime_error{"Cannot form a flock with zero boids"};
    }
    if (d_ <= 0.) {
      throw std::runtime_error{"Interaction distance (d) must be positive"};
    }
    if (ds_ <= 0. || ds_ >= d_) {
      throw std::runtime_error{
          "Separation distance (ds) must be positive and smaller than "
          "interaction distance (ds)"};
    }
    if (s_ <= 0. || s_ >= 1.) {
      throw std::runtime_error{
          "Separation parameter (s) must be between 0 and 1"};
    }
    if (a_ <= 0. || a_ >= 1.) {
      throw std::runtime_error{
          "Alignment parameter (a) must be between 0 and 1"};
    }
    if (c_ <= 0. || c_ >= 1.) {
      throw std::runtime_error{
          "Cohesion parameter (c) must be between 0 and 1"};
    }
  }

  inline const std::vector<Boid>& get_boids() const { return boids_; }
  inline double get_d() const { return d_; }
  inline double get_ds() const { return ds_; }
  inline double get_s() const { return s_; }
  inline double get_a() const { return a_; }
  inline double get_c() const { return c_; }
  inline Vector get_v1(Boid const& b) const { return v1_(b); }
  inline Vector get_v2(Boid const& b) const { return v2_(b); }
  inline Vector get_v3(Boid const& b) const { return v3_(b); }
  static constexpr int get_width() { return width_; }
  static constexpr int get_height() { return height_; }
  static constexpr double get_minspeed() { return min_speed_; }
  static constexpr double get_maxspeed() { return max_speed_; }

  size_t size() const;

  static void speed_limit(Boid&);

  static void bounce(Boid&);

  void update_neighbours();

  void update();
};

std::vector<Boid> generate_random_boids(size_t const);

void update(std::vector<Flock>&);

double mean_distance(Flock const&);

double sem_distance(Flock const&);

double mean_speed(Flock const&);

double sem_speed(Flock const&);

void run_simulation(std::vector<Flock>&);

}  // namespace pf

#endif