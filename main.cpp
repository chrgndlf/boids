#include <cstdlib>
#include <iostream>
#include <string>

#include "boids.hpp"

template <typename T>
T get_input(std::string const& s) {
  std::cout << s;
  T n;
  if (!(std::cin >> n)) {
    throw std::runtime_error{"Invalid input"};
  }
  return n;
}

int main() {
  try {
    std::cout << "--- Multi-Flock Simulation Setup ---\n";
    auto const n = get_input<size_t>("Enter number of flocks (n): ");
    std::vector<pf::Flock> flocks;
    for (size_t i{}; i < n; ++i) {
      std::cout << "\nFlock #" << i + 1 << "\n";
      auto const N = get_input<size_t>("Enter number of boids (N): ");
      auto const d = get_input<double>("Enter interaction distance (d): ");
      auto const ds = get_input<double>("Enter separation distance (ds): ");
      auto const s = get_input<double>("Enter separation parameter (s): ");
      auto const a = get_input<double>("Enter alignment parameter (a): ");
      auto const c = get_input<double>("Enter cohesion parameter (c): ");

      flocks.push_back(pf::Flock{pf::generate_random_boids(N), d, ds, s, a, c});
    }

    std::cout << "\nStarting simulation...\n";
    pf::run_simulation(flocks);
    std::cout << "--- End Simulation ---\n";
  } catch (std::exception const& e) {
    std::cerr << "Caught exception: '" << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}