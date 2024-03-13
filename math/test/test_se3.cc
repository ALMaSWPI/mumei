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
  EXPECT_TRUE(huron::SE3<double>().matrix().isApprox(
                Eigen::Matrix4d::Identity()));
}

TEST_F(TestSe3, InitFromMatrix) {
  Eigen::Isometry3d tf_gt = Eigen::Isometry3d::Identity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  EXPECT_TRUE(huron::SE3<double>(tf_gt.matrix()).matrix().isApprox(
                tf_gt.matrix()));
}

TEST_F(TestSe3, InitFromRotationTranslation) {
  Eigen::Isometry3d tf_gt = Eigen::Isometry3d::Identity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  huron::SE3<double> tf(tf_gt.rotation().matrix(),
                        tf_gt.translation().matrix());
  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Comparison) {
  Eigen::Isometry3d tf_gt = Eigen::Isometry3d::Identity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  huron::SE3<double> tf(tf_gt.matrix());
  huron::SE3<double> tf2(tf_gt.matrix());
  huron::SE3<double> tf3;
  EXPECT_TRUE(tf == tf2);
  EXPECT_FALSE(tf == tf3);
  EXPECT_FALSE(tf != tf2);
  EXPECT_TRUE(tf != tf3);
}

TEST_F(TestSe3, Assignment) {
  Eigen::Isometry3d tf_gt = Eigen::Isometry3d::Identity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  huron::SE3<double> tf(tf_gt.matrix());
  huron::SE3<double> tf2, tf3;
  EXPECT_TRUE(tf2 == tf3);
  tf2 = tf;
  EXPECT_TRUE(tf2 != tf3);
  EXPECT_TRUE(tf2.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Translation) {
  // Creating a random ground truth transformation
  Eigen::Isometry3d tf_gt = Eigen::Isometry3d::Identity();
  Eigen::Vector3d t(uniform_dist(re), uniform_dist(re), uniform_dist(re));
  tf_gt.translate(t);
  // Test object
  huron::SE3<double> tf;
  tf.Translate(t);

  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Prerotation) {
  // Creating a random ground truth transformation
  Eigen::Isometry3d a, b, tf_gt;
  a.setIdentity();
  b.setIdentity();
  tf_gt.setIdentity();
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
  tf.Prerotate(b.rotation().matrix());

  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Rotation) {
  // Creating a random ground truth transformation
  Eigen::Isometry3d a, b, tf_gt;
  a.setIdentity();
  b.setIdentity();
  tf_gt.setIdentity();
  std::cout << b.matrix() << std::endl;
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
  tf.Rotate(b.rotation());

  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, TransRot) {
  // Creating a random ground truth transformation
  Eigen::Isometry3d tf_gt;
  tf_gt.setIdentity();
  Eigen::Matrix3d R1 = Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Matrix3d R2 = Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Matrix3d R3 = Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d t(uniform_dist(re), uniform_dist(re), uniform_dist(re));
  tf_gt.rotate(R1);
  tf_gt.rotate(R2);
  tf_gt.rotate(R3);
  tf_gt.translate(t);
  // Test object
  huron::SE3<double> tf;
  tf.Rotate(R1);
  tf.Rotate(R2);
  tf.Rotate(R3);
  tf.Translate(t);

  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Multiplication) {
  // Creating a random ground truth transformation
  Eigen::Isometry3d a, b, tf_gt;
  a.setIdentity();
  b.setIdentity();
  tf_gt.setIdentity();
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

  EXPECT_TRUE(tf.matrix().isApprox(tf_gt.matrix()));
  EXPECT_TRUE((tfa *= tfb).matrix().isApprox(tf_gt.matrix()));
}

TEST_F(TestSe3, Inverse) {
  Eigen::Isometry3d tf_gt;
  tf_gt.setIdentity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  huron::SE3<double> tf(tf_gt.matrix());
  EXPECT_TRUE(tf.Inverse().matrix().isApprox(tf_gt.inverse().matrix()));
}

TEST_F(TestSe3, TransformVector) {
  Eigen::Isometry3d tf_gt;
  tf_gt.setIdentity();
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitX()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitY()));
  tf_gt.rotate(Eigen::AngleAxisd(uniform_dist(re), Eigen::Vector3d::UnitZ()));
  tf_gt.translate(Eigen::Vector3d(uniform_dist(re), uniform_dist(re), uniform_dist(re)));
  huron::SE3<double> tf(tf_gt.matrix());
  Eigen::Vector3d v;
  v << uniform_dist(re), uniform_dist(re), uniform_dist(re);
  EXPECT_TRUE((tf * v).isApprox(tf_gt * v));
}

TEST_F(TestSe3, TransformWrench) {
  Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
  tf.topRightCorner<1, 1>() << -6.0;
  huron::Vector6d w;
  w << 0.0, -5.0, 0.0, 0.0, 0.0, 0.0;

  huron::Vector6d expected;
  expected << 0.0, -5.0, 0.0, 0.0, 0.0, 30.0;

  huron::SE3<double> se3(tf);
  huron::Vector6d computed = se3.Inverse().AdjointAction().transpose() * w;

  EXPECT_TRUE(computed.isApprox(expected));
}
