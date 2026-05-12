#include "boids.hpp"

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>

namespace pf {

Vector& Vector::operator+=(Vector const& rhs) {
  x += rhs.x;
  y += rhs.y;
  return *this;
}
Vector& Vector::operator-=(Vector const& rhs) {
  x -= rhs.x;
  y -= rhs.y;
  return *this;
}
Vector& Vector::operator*=(double const scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}
Vector& Vector::operator/=(double const scalar) {
  if (scalar == 0.) {
    throw std::runtime_error{"Vector division by zero"};
  }
  x /= scalar;
  y /= scalar;
  return *this;
}
Vector operator+(Vector const& lhs, Vector const& rhs) {
  Vector result{lhs};
  return result += rhs;
}
Vector operator-(Vector const& lhs, Vector const& rhs) {
  Vector result{lhs};
  return result -= rhs;
}
Vector operator*(Vector const& v, double const scalar) {
  Vector result{v};
  return result *= scalar;
}
Vector operator*(double const scalar, Vector const& v) { return v * scalar; }
Vector operator/(Vector const& v, double const scalar) {
  Vector result{v};
  return result /= scalar;
}
bool operator==(Vector const& lhs, Vector const& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}
bool operator!=(Vector const& lhs, Vector const& rhs) { return !(lhs == rhs); }

double magnitude(Vector const& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

Vector normalize(Vector const& v) {
  auto const mag = magnitude(v);
  return (mag == 0.) ? Vector{0., 0.} : v / mag;
}

Vector Flock::v1_(Boid const& b_i) const {
  Vector sum{0., 0.};
  for (auto const* b_j : b_i.neighbors) {
    if (magnitude(b_i.p - b_j->p) < ds_) {
      sum += (b_j->p - b_i.p);
    }
  }
  return -s_ * sum;
}

Vector Flock::v2_(Boid const& b_i) const {
  if (b_i.neighbors.empty()) {
    return Vector{0., 0.};
  }
  Vector sum = std::accumulate(
      b_i.neighbors.begin(), b_i.neighbors.end(), Vector{0., 0.},
      [](Vector acc, Boid const* b_j) { return acc + b_j->v; });
  return a_ * ((sum / static_cast<double>(b_i.neighbors.size())) - b_i.v);
}

Vector Flock::v3_(Boid const& b_i) const {
  if (b_i.neighbors.empty()) {
    return Vector{0., 0.};
  }
  Vector sum = std::accumulate(
      b_i.neighbors.begin(), b_i.neighbors.end(), Vector{0., 0.},
      [](Vector acc, Boid const* b_j) { return acc + b_j->p; });
  Vector center_of_mass{sum / static_cast<double>(b_i.neighbors.size())};
  return c_ * (center_of_mass - b_i.p);
}

size_t Flock::size() const { return boids_.size(); }

void Flock::speed_limit(Boid& b) {
  auto const mag = magnitude(b.v);
  if (mag < min_speed_) {
    if (mag == 0.) {
      static std::random_device rd;
      static std::default_random_engine eng(rd());
      static std::uniform_real_distribution<double> dist_rad{
          0., 2. * pf::costants::pi};
      auto const alpha = dist_rad(eng);
      b.v = Vector{std::cos(alpha), std::sin(alpha)} * min_speed_;
    } else {
      b.v = normalize(b.v) * min_speed_;
    }
  }
  if (mag > max_speed_) {
    b.v = normalize(b.v) * max_speed_;
  }
}

void Flock::bounce(Boid& b) {
  if (b.p.x < 0.) {
    b.p.x = 0.;
    b.v.x = std::abs(b.v.x);
  }
  if (b.p.x > width_) {
    b.p.x = static_cast<double>(width_);
    b.v.x = -std::abs(b.v.x);
  }
  if (b.p.y < 0.) {
    b.p.y = 0.;
    b.v.y = std::abs(b.v.y);
  }
  if (b.p.y > height_) {
    b.p.y = static_cast<double>(height_);
    b.v.y = -std::abs(b.v.y);
  }
}

void Flock::update_neighbours() {
  for (auto& b_i : boids_) {
    b_i.neighbors.clear();
    for (auto const& b_j : boids_) {
      if (&b_i != &b_j && magnitude(b_i.p - b_j.p) < d_) {
        b_i.neighbors.push_back(&b_j);
      }
    }
  }
}

void Flock::update() {
  update_neighbours();
  auto const N = boids_.size();
  std::vector<Vector> new_v(N);
  for (size_t i{}; i < N; ++i) {
    new_v[i] =
        boids_[i].v + get_v1(boids_[i]) + get_v2(boids_[i]) + get_v3(boids_[i]);
  }
  for (size_t i{}; i < N; ++i) {
    boids_[i].v = new_v[i];
    speed_limit(boids_[i]);
    boids_[i].p += boids_[i].v;  // *Δt = 1s
    bounce(boids_[i]);
  }
}

std::vector<Boid> generate_random_boids(size_t const N) {
  if (N == 0) {
    throw std::runtime_error{"Cannot generate zero boids"};
  }
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> dist_x{
      0., static_cast<double>(Flock::get_width())};
  std::uniform_real_distribution<double> dist_y{
      0., static_cast<double>(Flock::get_height())};
  std::uniform_real_distribution<double> dist_v{Flock::get_minspeed(),
                                                Flock::get_maxspeed()};
  std::uniform_real_distribution<double> dist_rad{0., 2. * pf::costants::pi};
  std::vector<Boid> boids(N);
  for (auto& b : boids) {
    auto const v = dist_v(eng);
    auto const alpha = dist_rad(eng);
    b.p = {dist_x(eng), dist_y(eng)};
    b.v = {v * std::cos(alpha), v * std::sin(alpha)};
  }
  return boids;
}

void update(std::vector<Flock>& flocks) {
  for (auto& f : flocks) {
    f.update();
  }
}

double mean_distance(Flock const& f) {
  auto const N = f.size();
  if (N < 2) {
    throw std::runtime_error{
        "Cannot compute mean distance (minimum 2 boids required)"};
  }
  double sum_d{};
  auto const& boids = f.get_boids();
  auto const pairs = static_cast<double>(N * (N - 1)) / 2.;
  for (size_t i{}; i < N; ++i) {
    for (size_t j{i + 1}; j < N; ++j) {
      sum_d += magnitude(boids[i].p - boids[j].p);
    }
  }
  return sum_d / pairs;
}

double sem_distance(Flock const& f) {
  auto const N = f.size();
  if (N < 3) {
    throw std::runtime_error{
        "Cannot compute standard error of the mean distance (minimum 3 boids "
        "required)"};
  }
  double sum_d2{};
  auto const& boids = f.get_boids();
  auto const d = mean_distance(f);
  auto const pairs = static_cast<double>(N * (N - 1)) / 2.;
  for (size_t i{}; i < N; ++i) {
    for (size_t j{i + 1}; j < N; ++j) {
      sum_d2 += std::pow(magnitude(boids[i].p - boids[j].p) - d, 2);
    }
  }
  return std::sqrt(sum_d2 / (pairs * (pairs - 1.)));
}

double mean_speed(Flock const& f) {
  auto const N = f.size();
  if (N == 0) {
    throw std::runtime_error{
        "Cannot compute mean speed (minimum 1 boid required)"};
  }
  auto const& boids = f.get_boids();
  double sum_v = std::accumulate(
      boids.begin(), boids.end(), 0.,
      [](double acc, const Boid& b) { return acc + magnitude(b.v); });
  return sum_v / static_cast<double>(N);
}

double sem_speed(Flock const& f) {
  auto const N = f.size();
  if (N < 2) {
    throw std::runtime_error{
        "Cannot compute standard error of the mean speed (minimum 2 boids "
        "required)"};
  }
  const auto& boids = f.get_boids();
  auto const v = mean_speed(f);
  double sum_v2 = std::accumulate(
      boids.begin(), boids.end(), 0., [v](double acc, const Boid& b) {
        return acc + std::pow(magnitude(b.v) - v, 2);
      });
  return std::sqrt(sum_v2 / static_cast<double>(N * (N - 1)));
}

void run_simulation(std::vector<Flock>& flocks) {
  if (flocks.empty()) {
    throw std::runtime_error{
        "Cannot run simulation (minimum 1 flock required)"};
  }

  // Layout
  auto const simulation_width = static_cast<float>(Flock::get_width());
  auto const simulation_height = static_cast<float>(Flock::get_height());
  float const stats_width{240.f};
  auto const width = simulation_width + stats_width;
  auto const height = simulation_height;
  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(width),
                                        static_cast<unsigned int>(height)),
                          "Multi-Flock Simulation",
                          sf::Style::Titlebar | sf::Style::Close);
  window.setFramerateLimit(60);

  sf::View simulation_view(
      sf::FloatRect(0.f, 0.f, simulation_width, simulation_height));
  simulation_view.setViewport(
      sf::FloatRect(0.f, 0.f, simulation_width / width, 1.f));
  sf::View stats_view(sf::FloatRect(0.f, 0.f, stats_width, simulation_height));
  stats_view.setViewport(
      sf::FloatRect(simulation_width / width, 0.f, stats_width / width, 1.f));
  sf::Vector2f const stats_view_center = stats_view.getCenter();

  sf::RectangleShape background;
  background.setSize(sf::Vector2f(simulation_width, simulation_height));
  background.setFillColor(sf::Color(135, 206, 235));
  sf::RectangleShape stats_background(
      sf::Vector2f(stats_width, simulation_height));
  stats_background.setFillColor(sf::Color::Black);
  sf::RectangleShape separator_line(sf::Vector2f(2.f, simulation_height));
  separator_line.setFillColor(sf::Color::White);
  separator_line.setPosition(0.f, 0.f);
  sf::RectangleShape scroll_bar;
  scroll_bar.setFillColor(sf::Color(80, 80, 80));
  scroll_bar.setSize(sf::Vector2f(6.f, 0.f));

  sf::Texture boid_texture;
  if (!boid_texture.loadFromFile("boid_sprite.png")) {
    throw std::runtime_error{"Cannot load 'boid_sprite.png'"};
  }
  boid_texture.setSmooth(true);
  const int n_frames{4};
  auto const frame_width =
      static_cast<int>(boid_texture.getSize().x) / n_frames;
  auto const frame_height = static_cast<int>(boid_texture.getSize().y);
  sf::Sprite boid_sprite(boid_texture);
  boid_sprite.setScale(0.08f, 0.08f);
  boid_sprite.setOrigin(static_cast<float>(frame_width) / 2.f,
                        static_cast<float>(frame_height) / 2.f);
  std::vector<sf::Color> colors = {sf::Color::Green, sf::Color::Yellow,
                                   sf::Color::Red, sf::Color::Magenta,
                                   sf::Color::Blue};

  sf::Font font;
  if (!font.loadFromFile("segoeui.ttf")) {
    throw std::runtime_error{"Cannot load 'segoeui.ttf'"};
  }
  sf::Text stats_text;
  stats_text.setFont(font);
  stats_text.setCharacterSize(11);
  stats_text.setFillColor(sf::Color::White);
  stats_text.setPosition(10.f, 10.f);

  sf::Clock clock;
  sf::Clock stats_timer;
  int current_frame{};
  float frame_time{0.33f};
  std::string stats_string{"Calculating stats..."};
  float scroll_offset{0.f};
  float maxscroll_offset{0.f};

  // Update
  while (window.isOpen()) {
    sf::Event event{};

    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Escape) {
          window.close();
        }
        if (event.key.code == sf::Keyboard::Up) {
          scroll_offset -= 40.f;
        }
        if (event.key.code == sf::Keyboard::Down) {
          scroll_offset += 40.f;
        }
      }
      if (event.type == sf::Event::MouseWheelScrolled) {
        if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel) {
          scroll_offset -= event.mouseWheelScroll.delta * 40.f;
        }
      }
    }

    if (clock.getElapsedTime().asSeconds() > frame_time) {
      current_frame = (current_frame + 1) % n_frames;
      clock.restart();
    }
    boid_sprite.setTextureRect(
        sf::IntRect(current_frame * frame_width, 0, frame_width, frame_height));

    update(flocks);

    if (stats_timer.getElapsedTime().asSeconds() >= 1.f) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(3);
      ss << "Statistics:\n";
      auto const N = flocks.size();
      for (size_t i{}; i < N; ++i) {
        ss << "\nFlock #" << i + 1 << "\n";
        ss << "Boids: " << flocks[i].size() << "\n";
        ss << "Distance: (" << mean_distance(flocks[i]) << " +/- "
           << sem_distance(flocks[i]) << ") px\n";
        ss << "Speed: (" << mean_speed(flocks[i]) << " +/- "
           << sem_speed(flocks[i]) << ") px/frame\n";
      }
      stats_string = ss.str();
      stats_timer.restart();
    }
    stats_text.setString(stats_string);

    auto const text_height = stats_text.getGlobalBounds().height + 40.f;
    maxscroll_offset = std::max(0.f, text_height - simulation_height);

    if (scroll_offset < 0.f) {
      scroll_offset = 0.f;
    }
    if (scroll_offset > maxscroll_offset) {
      scroll_offset = maxscroll_offset;
    }
    stats_view.setCenter(stats_view_center.x,
                         stats_view_center.y + scroll_offset);
    if (maxscroll_offset > 0.f) {
      float view_ratio = simulation_height / text_height;
      float bar_height = simulation_height * view_ratio;
      if (bar_height < 30.f) {
        bar_height = 30.f;
      }
      auto const scroll_percent = scroll_offset / maxscroll_offset;
      auto const bar_space = simulation_height - bar_height;
      auto const bar_y = (stats_view.getCenter().y - simulation_height / 2.f) +
                         (scroll_percent * bar_space);
      scroll_bar.setSize(sf::Vector2f(6.f, bar_height));
      scroll_bar.setPosition(stats_width - 8.f, bar_y);
    }

    // Drawing
    window.clear(sf::Color::Black);

    window.setView(simulation_view);
    window.draw(background);
    auto const N = flocks.size();
    for (size_t i{}; i < N; ++i) {
      boid_sprite.setColor(colors[i % colors.size()]);
      for (auto const& b : flocks[i].get_boids()) {
        boid_sprite.setPosition(static_cast<float>(b.p.x),
                                static_cast<float>(b.p.y));
        auto const angle = static_cast<float>(std::atan2(b.v.y, b.v.x) * 180. /
                                              pf::costants::pi);
        boid_sprite.setRotation(angle);
        window.draw(boid_sprite);
      }
    }

    window.setView(stats_view);
    stats_background.setPosition(
        0.f, stats_view.getCenter().y - simulation_height / 2.f);
    separator_line.setPosition(
        0.f, stats_view.getCenter().y - simulation_height / 2.f);
    window.draw(stats_background);
    window.draw(stats_text);
    window.draw(separator_line);

    if (maxscroll_offset > 0.f) {
      window.draw(scroll_bar);
    }

    window.display();
  }
}
}  // namespace pf