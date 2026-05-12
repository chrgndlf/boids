#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Testing Vector") {
  pf::Vector v0{0., 0.};
  pf::Vector v1{3., 4.};
  pf::Vector v2{2., 3.};

  SUBCASE("Testing addition") {
    auto const& sum = v1 + v2;
    CHECK(sum.x == doctest::Approx(5.));
    CHECK(sum.y == doctest::Approx(7.));
  }

  SUBCASE("Testing subtraction") {
    auto const& diff = v1 - v2;
    CHECK(diff.x == doctest::Approx(1.));
    CHECK(diff.y == doctest::Approx(1.));
  }

  SUBCASE("Testing scalar multiplication") {
    auto const& mult1 = v1 * 2.;
    auto const& mult2 = 2. * v1;
    CHECK(mult1.x == doctest::Approx(6.));
    CHECK(mult1.y == doctest::Approx(8.));
    CHECK(mult2.x == doctest::Approx(6.));
    CHECK(mult2.y == doctest::Approx(8.));
  }

  SUBCASE("Testing scalar division") {
    auto const& div = v1 / 2.;
    CHECK_THROWS(v1 / 0.);
    CHECK(div.x == doctest::Approx(1.5));
    CHECK(div.y == doctest::Approx(2.));
  }

  SUBCASE("Testing equality and inequality") {
    pf::Vector v3{3., 4.};
    CHECK(v1 == v3);
    CHECK(v1 != v2);
  }

  SUBCASE("Testing magnitude") {
    CHECK(magnitude(v0) == doctest::Approx(0.));
    CHECK(magnitude(v1) == doctest::Approx(5.));
  }

  SUBCASE("Testing normalize") {
    auto const n0 = normalize(v0);
    auto const n1 = normalize(v1);
    CHECK(magnitude(n0) == doctest::Approx(0.));
    CHECK(magnitude(n1) == doctest::Approx(1.));
  }
}

TEST_CASE("Testing Flock") {
  std::vector<pf::Boid> boids{};
  double const d{50.};
  double const ds{20.};
  double const s{0.1};
  double const a{0.1};
  double const c{0.1};

  SUBCASE("Testing constructor") {
    pf::Boid b{{0., 0.}, {0., 0.}, {}};
    CHECK_THROWS(pf::Flock{boids, d, ds, s, a, c});
    boids.push_back(b);
    CHECK_THROWS(pf::Flock(boids, -50., ds, s, a, c));
    CHECK_THROWS(pf::Flock(boids, d, -20., s, a, c));
    CHECK_THROWS(pf::Flock(boids, d, 70., s, a, c));
    CHECK_THROWS(pf::Flock(boids, d, ds, -0.1, a, c));
    CHECK_THROWS(pf::Flock(boids, d, ds, 1.1, a, c));
    CHECK_THROWS(pf::Flock(boids, d, ds, s, -0.1, c));
    CHECK_THROWS(pf::Flock(boids, d, ds, s, 1.1, c));
    CHECK_THROWS(pf::Flock(boids, d, ds, s, a, -0.1));
    CHECK_THROWS(pf::Flock(boids, d, ds, s, a, 1.1));
  }

  SUBCASE("Testing speed_limit") {
    pf::Boid b1{{0., 0.}, {0., 0.}, {}};
    pf::Boid b2{{0., 0.}, {1., 1.}, {}};
    pf::Boid b3{{0., 0.}, {5., 5.}, {}};
    pf::Flock::speed_limit(b1);
    pf::Flock::speed_limit(b2);
    pf::Flock::speed_limit(b3);
    CHECK(magnitude(b1.v) == doctest::Approx(pf::Flock::get_minspeed()));
    CHECK(magnitude(b2.v) == doctest::Approx(pf::Flock::get_minspeed()));
    CHECK(magnitude(b3.v) == doctest::Approx(pf::Flock::get_maxspeed()));
  }

  SUBCASE("Testing bounce") {
    pf::Boid b1{{-10., -10.}, {-1., -1.}, {}};
    pf::Boid b2{{810., 610.}, {1., 1.}, {}};
    pf::Flock::bounce(b1);
    pf::Flock::bounce(b2);
    CHECK(b1.p.x == doctest::Approx(0.));
    CHECK(b1.p.y == doctest::Approx(0.));
    CHECK(b1.v.x == doctest::Approx(1.));
    CHECK(b1.v.y == doctest::Approx(1.));
    CHECK(b2.p.x == doctest::Approx(pf::Flock::get_width()));
    CHECK(b2.p.y == doctest::Approx(pf::Flock::get_height()));
    CHECK(b2.v.x == doctest::Approx(-1.));
    CHECK(b2.v.y == doctest::Approx(-1.));
  }

  SUBCASE("Testing update_neighbours - one boid") {
    pf::Boid b{{0., 0.}, {0., 0.}, {}};
    boids.push_back(b);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(fb[0].neighbors.size() == 0);
  }

  SUBCASE("Testing update_neighbours - more boids") {
    pf::Boid b1{{0., 0.}, {0., 0.}, {}};
    pf::Boid b2{{0., 0.}, {0., 0.}, {}};
    pf::Boid b3{{d, 0.}, {0., 0.}, {}};
    pf::Boid b4{{100., 100.}, {0., 0.}, {}};
    pf::Boid b5{{110., 110.}, {0., 0.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    boids.push_back(b5);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(fb[0].neighbors.size() == 1);
    CHECK(fb[0].neighbors[0] == &fb[1]);
    CHECK(fb[1].neighbors.size() == 1);
    CHECK(fb[1].neighbors[0] == &fb[0]);
    CHECK(fb[2].neighbors.size() == 0);
    CHECK(fb[3].neighbors.size() == 1);
    CHECK(fb[3].neighbors[0] == &fb[4]);
    CHECK(fb[4].neighbors.size() == 1);
    CHECK(fb[4].neighbors[0] == &fb[3]);
  }

  SUBCASE("Testing v1)") {
    pf::Boid b1{{0., 0.}, {3., 4.}, {}};
    pf::Boid b2{{ds, 0.}, {6., 8.}, {}};
    pf::Boid b3{{100., 100.}, {3., 4.}, {}};
    pf::Boid b4{{105., 105.}, {6., 8.}, {}};
    pf::Boid b5{{110., 110.}, {12., 16.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    boids.push_back(b5);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(pf::magnitude(f.get_v1(fb[0])) == doctest::Approx(0.));
    CHECK(pf::magnitude(f.get_v1(fb[1])) == doctest::Approx(0.));
    CHECK(f.get_v1(fb[2]).x == doctest::Approx(-1.5));
    CHECK(f.get_v1(fb[2]).y == doctest::Approx(-1.5));
    CHECK(pf::magnitude(f.get_v1(fb[3])) == doctest::Approx(0.));
  }

  SUBCASE("Testing v2") {
    pf::Boid b1{{0., 0.}, {3., 4.}, {}};
    pf::Boid b2{{100., 100.}, {3., 4.}, {}};
    pf::Boid b3{{110., 110.}, {6., 8.}, {}};
    pf::Boid b4{{120., 120.}, {12., 16.}, {}};
    pf::Boid b5{{130., 130.}, {24., 32.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    boids.push_back(b5);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(pf::magnitude(f.get_v2(fb[0])) == doctest::Approx(0.));
    CHECK(f.get_v2(fb[1]).x == doctest::Approx(1.1));
    CHECK(f.get_v2(fb[1]).y == doctest::Approx(1.4666).epsilon(0.0001));
    CHECK(f.get_v2(fb[2]).x == doctest::Approx(0.7));
    CHECK(f.get_v2(fb[2]).y == doctest::Approx(0.9333).epsilon(0.0001));
    CHECK(f.get_v2(fb[3]).x == doctest::Approx(-0.1));
    CHECK(f.get_v2(fb[3]).y == doctest::Approx(-0.1333).epsilon(0.0001));
    CHECK(f.get_v2(fb[4]).x == doctest::Approx(-1.7));
    CHECK(f.get_v2(fb[4]).y == doctest::Approx(-2.2666).epsilon(0.0001));
  }

  SUBCASE("Testing v3") {
    pf::Boid b1{{0., 0.}, {3., 4.}, {}};
    pf::Boid b2{{100., 100.}, {3., 4.}, {}};
    pf::Boid b3{{110., 110.}, {6., 8.}, {}};
    pf::Boid b4{{120., 120.}, {12., 16.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(pf::magnitude(f.get_v3(fb[0])) == doctest::Approx(0.));
    CHECK(f.get_v3(fb[1]).x == doctest::Approx(1.5));
    CHECK(f.get_v3(fb[1]).y == doctest::Approx(1.5));
    CHECK(pf::magnitude(f.get_v3(fb[2])) == doctest::Approx(0.));
    CHECK(f.get_v3(fb[3]).x == doctest::Approx(-1.5));
    CHECK(f.get_v3(fb[3]).y == doctest::Approx(-1.5));
  }

  SUBCASE("Testing update - one boid") {
    pf::Boid b1{{0., 0.}, {3., 4.}, {}};
    boids.push_back(b1);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update();
    auto const& fb = f.get_boids();
    CHECK(pf::magnitude(f.get_v1(fb[0])) == doctest::Approx(0.));
    CHECK(pf::magnitude(f.get_v2(fb[0])) == doctest::Approx(0.));
    CHECK(pf::magnitude(f.get_v3(fb[0])) == doctest::Approx(0.));
    CHECK(fb[0].v.x == doctest::Approx(3.));
    CHECK(fb[0].v.y == doctest::Approx(4.));
    CHECK(fb[0].p.x == doctest::Approx(3.));
    CHECK(fb[0].p.y == doctest::Approx(4.));
  }

  SUBCASE("Testing update - more boids") {
    pf::Boid b1{{-10., -10.}, {0., 0.}, {}};
    pf::Boid b2{{0., 0.}, {1., 1.}, {}};
    pf::Boid b3{{10., 10.}, {2., 2.}, {}};
    pf::Boid b4{{20., 20.}, {3., 3.}, {}};
    pf::Boid b5{{100., 100.}, {4., 4.}, {}};
    pf::Boid b6{{110., 110.}, {5., 5.}, {}};
    pf::Boid b7{{810., 610.}, {6., 6.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    boids.push_back(b5);
    boids.push_back(b6);
    boids.push_back(b7);
    pf::Flock f{boids, d, ds, s, a, c};
    f.update_neighbours();
    auto const& fb = f.get_boids();
    CHECK(f.get_v1(fb[0]).x == doctest::Approx(-1.));
    CHECK(f.get_v1(fb[0]).y == doctest::Approx(-1.));
    CHECK(f.get_v2(fb[0]).x == doctest::Approx(0.2));
    CHECK(f.get_v2(fb[0]).y == doctest::Approx(0.2));
    CHECK(f.get_v3(fb[0]).x == doctest::Approx(2.));
    CHECK(f.get_v3(fb[0]).y == doctest::Approx(2.));
    CHECK(pf::magnitude(f.get_v1(fb[1])) == doctest::Approx(0.));
    CHECK(f.get_v2(fb[1]).x == doctest::Approx(0.0666).epsilon(0.0001));
    CHECK(f.get_v2(fb[1]).y == doctest::Approx(0.0666).epsilon(0.0001));
    CHECK(f.get_v3(fb[1]).x == doctest::Approx(0.6666).epsilon(0.0001));
    CHECK(f.get_v3(fb[1]).y == doctest::Approx(0.6666).epsilon(0.0001));
    CHECK(pf::magnitude(f.get_v1(fb[2])) == doctest::Approx(0.));
    CHECK(f.get_v2(fb[2]).x == doctest::Approx(-0.0666).epsilon(0.0001));
    CHECK(f.get_v2(fb[2]).y == doctest::Approx(-0.0666).epsilon(0.0001));
    CHECK(f.get_v3(fb[2]).x == doctest::Approx(-0.6666).epsilon(0.0001));
    CHECK(f.get_v3(fb[2]).y == doctest::Approx(-0.6666).epsilon(0.0001));
    CHECK(f.get_v1(fb[3]).x == doctest::Approx(1.));
    CHECK(f.get_v1(fb[3]).y == doctest::Approx(1.));
    CHECK(f.get_v2(fb[3]).x == doctest::Approx(-0.2));
    CHECK(f.get_v2(fb[3]).y == doctest::Approx(-0.2));
    CHECK(f.get_v3(fb[3]).x == doctest::Approx(-2.));
    CHECK(f.get_v3(fb[3]).y == doctest::Approx(-2.));
    CHECK(f.get_v1(fb[4]).x == doctest::Approx(-1.));
    CHECK(f.get_v1(fb[4]).y == doctest::Approx(-1.));
    CHECK(f.get_v2(fb[4]).x == doctest::Approx(0.1));
    CHECK(f.get_v2(fb[4]).y == doctest::Approx(0.1));
    CHECK(f.get_v3(fb[4]).x == doctest::Approx(1.));
    CHECK(f.get_v3(fb[4]).y == doctest::Approx(1.));
    CHECK(f.get_v1(fb[5]).x == doctest::Approx(1.));
    CHECK(f.get_v1(fb[5]).y == doctest::Approx(1.));
    CHECK(f.get_v2(fb[5]).x == doctest::Approx(-0.1));
    CHECK(f.get_v2(fb[5]).y == doctest::Approx(-0.1));
    CHECK(f.get_v3(fb[5]).x == doctest::Approx(-1.));
    CHECK(f.get_v3(fb[5]).y == doctest::Approx(-1.));
    CHECK(pf::magnitude(f.get_v1(fb[6])) == doctest::Approx(0.));
    CHECK(pf::magnitude(f.get_v2(fb[6])) == doctest::Approx(0.));
    CHECK(pf::magnitude(f.get_v3(fb[6])) == doctest::Approx(0.));
    f.update();
    CHECK(fb[0].v.x == doctest::Approx(1.4142));
    CHECK(fb[0].v.y == doctest::Approx(1.4142));
    CHECK(fb[0].p.x == doctest::Approx(0.));
    CHECK(fb[0].p.y == doctest::Approx(0.));
    CHECK(fb[1].v.x == doctest::Approx(1.7333).epsilon(0.0001));
    CHECK(fb[1].v.y == doctest::Approx(1.7333).epsilon(0.0001));
    CHECK(fb[1].p.x == doctest::Approx(1.7333).epsilon(0.0001));
    CHECK(fb[1].p.y == doctest::Approx(1.7333).epsilon(0.0001));
    CHECK(fb[2].v.x == doctest::Approx(1.4142));
    CHECK(fb[2].v.y == doctest::Approx(1.4142));
    CHECK(fb[2].p.x == doctest::Approx(11.4142));
    CHECK(fb[2].p.y == doctest::Approx(11.4142));
    CHECK(fb[3].v.x == doctest::Approx(1.8));
    CHECK(fb[3].v.y == doctest::Approx(1.8));
    CHECK(fb[3].p.x == doctest::Approx(21.8));
    CHECK(fb[3].p.y == doctest::Approx(21.8));
    CHECK(fb[4].v.x == doctest::Approx(3.5355).epsilon(0.0001));
    CHECK(fb[4].v.y == doctest::Approx(3.5355).epsilon(0.0001));
    CHECK(fb[4].p.x == doctest::Approx(103.5355).epsilon(0.0001));
    CHECK(fb[4].p.y == doctest::Approx(103.5355).epsilon(0.0001));
    CHECK(fb[5].v.x == doctest::Approx(3.5355).epsilon(0.0001));
    CHECK(fb[5].v.y == doctest::Approx(3.5355).epsilon(0.0001));
    CHECK(fb[5].p.x == doctest::Approx(113.5355).epsilon(0.0001));
    CHECK(fb[5].p.y == doctest::Approx(113.5355).epsilon(0.0001));
    CHECK(fb[6].v.x == doctest::Approx(-3.5355).epsilon(0.0001));
    CHECK(fb[6].v.y == doctest::Approx(-3.5355).epsilon(0.0001));
    CHECK(fb[6].p.x == doctest::Approx(800.));
    CHECK(fb[6].p.y == doctest::Approx(600.));
  }

  SUBCASE("Testing statistics - one boid") {
    pf::Boid b{{0., 0.}, {3., 4.}, {}};
    boids.push_back(b);
    pf::Flock f{boids, d, ds, s, a, c};
    CHECK_THROWS(pf::mean_distance(f));
    CHECK(pf::mean_speed(f) == doctest::Approx(5.));
    CHECK_THROWS(pf::sem_distance(f));
    CHECK_THROWS(pf::sem_speed(f));
    f.update();
    CHECK_THROWS(pf::mean_distance(f));
    CHECK(pf::mean_speed(f) == doctest::Approx(5.));
    CHECK_THROWS(pf::sem_distance(f));
    CHECK_THROWS(pf::sem_speed(f));
  }

  SUBCASE("Testing statistics - two boids") {
    pf::Boid b1{{0., 0.}, {3., 4.}, {}};
    pf::Boid b2{{10., 10.}, {6., 8.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    pf::Flock f{boids, d, ds, s, a, c};
    CHECK(pf::mean_distance(f) == doctest::Approx(14.1421).epsilon(0.0001));
    CHECK_THROWS(pf::sem_distance(f));
    CHECK(pf::mean_speed(f) == doctest::Approx(7.5));
    CHECK(pf::sem_speed(f) == doctest::Approx(2.5));
    f.update();
    CHECK(pf::mean_distance(f) == doctest::Approx(14.1421).epsilon(0.0001));
    CHECK_THROWS(pf::sem_distance(f));
    CHECK(pf::mean_speed(f) == doctest::Approx(5.));
    CHECK(pf::sem_speed(f) == doctest::Approx(0.));
  }

  SUBCASE("Testing statistics - more boids") {
    pf::Boid b1{{0., 0.}, {1., 1.}, {}};
    pf::Boid b2{{10., 10.}, {2., 2.}, {}};
    pf::Boid b3{{20., 20.}, {3., 3.}, {}};
    pf::Boid b4{{100., 100.}, {4., 4.}, {}};
    pf::Boid b5{{110., 110.}, {5., 5.}, {}};
    boids.push_back(b1);
    boids.push_back(b2);
    boids.push_back(b3);
    boids.push_back(b4);
    boids.push_back(b5);
    pf::Flock f{boids, d, ds, s, a, c};
    CHECK(pf::mean_distance(f) == doctest::Approx(87.6812).epsilon(0.0001));
    CHECK(pf::sem_distance(f) == doctest::Approx(19.4136).epsilon(0.0001));
    CHECK(pf::mean_speed(f) == doctest::Approx(4.2426).epsilon(0.0001));
    CHECK(pf::sem_speed(f) == doctest::Approx(1.));
    f.update();
    CHECK(pf::mean_distance(f) == doctest::Approx(89.1820).epsilon(0.0001));
    CHECK(pf::sem_distance(f) == doctest::Approx(19.7020).epsilon(0.0001));
    CHECK(pf::mean_speed(f) == doctest::Approx(3.6971).epsilon(0.0001));
    CHECK(pf::sem_speed(f) == doctest::Approx(0.5545).epsilon(0.0001));
  }
}