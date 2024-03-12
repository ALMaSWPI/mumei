#include <random>
#include <gtest/gtest.h>
#include <eigen3/Eigen/Dense>
#include "huron/math/se3.h"

class TestSe3 : public testing::Test {
 protected:
  void SetUp() override {
    uniform_dist = std::uniform_real_distribution<double>(-3.14159, 3.14159);
  }

  // void TearDown() override {}
  std::uniform_real_distribution<double> uniform_dist;
  std::default_random_engine re;
};

TEST_F(TestSe3, Identity) {
  EXPECT_EQ(huron::SE3<double>().matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(TestSe3, InitFromMatrix) {
  Eigen::Affine3d tf_gt;
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  EXPECT_EQ(huron::SE3<double>(tf_gt.matrix()).matrix(), tf_gt.matrix());
}

TEST_F(TestSe3, InitFromRotationTranslation) {
  Eigen::Affine3d tf_gt;
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  EXPECT_EQ(huron::SE3<double>(tf_gt.rotation().matrix(),
                               tf_gt.translation().matrix()).matrix(),
            tf_gt.matrix());
}

TEST_F(TestSe3, Translation) {
  // Creating a random ground truth transformation
  Eigen::Affine3d tf_gt;
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  // Test object
  huron::SE3<double> tf;
  tf.translate(tf_gt.translation().matrix());

  EXPECT_EQ(tf.matrix(), tf_gt.matrix());
}

TEST_F(TestSe3, Prerotation) {
  // Creating a random ground truth transformation
  std::uniform_real_distribution<double> uniform_dist(-3.14159, 3.14159);
  std::default_random_engine re;
  Eigen::Affine3d a, b, tf_gt;
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt = a;
  tf_gt.prerotate(b.rotation());
  // Test object
  huron::SE3<double> tf(a.matrix());;
  tf.prerotate(b.rotation().matrix());

  EXPECT_EQ(tf.matrix(), tf_gt.matrix());
}

TEST_F(TestSe3, Rotation) {
  // Creating a random ground truth transformation
  std::uniform_real_distribution<double> uniform_dist(-3.14159, 3.14159);
  std::default_random_engine re;
  Eigen::Affine3d a, b, tf_gt;
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt = a;
  tf_gt.rotate(b.rotation());
  // Test object
  huron::SE3<double> tf(a.matrix());;
  tf.rotate(b.rotation().matrix());

  EXPECT_EQ(tf.matrix(), tf_gt.matrix());
}

TEST_F(TestSe3, TransRot) {
  // Creating a random ground truth transformation
  Eigen::Affine3d tf_gt;
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  // Test object
  huron::SE3<double> tf;
  tf.rotate(tf_gt.rotation().matrix());
  tf.translate(tf_gt.translation().matrix());

  EXPECT_EQ(tf.matrix(), tf_gt.matrix());
}

TEST_F(TestSe3, Multiplication) {
  // Creating a random ground truth transformation
  Eigen::Affine3d a, b, tf_gt;
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  a.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  a.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  b.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  b.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  tf_gt = a * b;
  // Test object
  huron::SE3<double> tfa(a.matrix()), tfb(b.matrix()), tf;
  tf = tfa * tfb;

  EXPECT_EQ(tf.matrix(), tf_gt.matrix());
  EXPECT_EQ((tfa *= tfb).matrix(), tf_gt.matrix());
}
