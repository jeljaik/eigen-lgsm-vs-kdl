// #include "this/package/foo.h"
#include "gtest/gtest.h"
#include "kdl/frames.hpp"
#include "kdl/frames_io.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Lgsm"

#define ABS_ERROR 1e-5
namespace {

    // The fixture for testing class Foo.
    class KDLvsLGSMTest : public ::testing::Test {
    protected:
        // You can remove any or all of the following functions if its body
        // is empty.

        // KDL Default constructor
        KDL::Vector kv1;
        KDL::Vector kv2;
        KDL::Vector kv3;
        KDL::Vector kv4;

        // Eigen Default constructors
        Eigen::Vector3d ev1;
        Eigen::Vector3d ev2;
        Eigen::Vector3d ev3;
        Eigen::Vector3d ev4;

        // KDL Rotations
        KDL::Rotation kr1; // Default constructor
        KDL::Rotation kr2;
        KDL::Rotation kr3;

        // LGSM Rotations
        Eigen::Rotation3d er1; // Default constructor

        // KDL frames
        KDL::Frame kfr1;
        // LGSM frames
        Eigen::Displacementd efr1;

        KDLvsLGSMTest() {
            // You can do set-up work for each test here.
        }

        virtual ~KDLvsLGSMTest() {
            // You can do clean-up work that doesn't throw exceptions here.
        }

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:

        virtual void SetUp() {
            // kv1 is by default initialized to zero
            // KDL Vectors are of size 3 by default
            kv2 = KDL::Vector(1.0,2.0,3.0);//Most used constructor
            kv3 = KDL::Vector(kv2);//Copy constructor
            kv4 = KDL::Vector::Zero();//Static member

            ev1.resize(3); ev1.setZero();
            ev2.resize(3);
            ev2 << 1.0,2.0,3.0;
            ev3 = Eigen::Vector3d(ev2);
            ev4 = Eigen::Vector3d::Zero(3);

        }

        // Compares (asserts) two equal vectors. KDL vs Eigen
        void assertEqualVectors(KDL::Vector kv, Eigen::Vector3d ev) {
            for (size_t i = 0; i < 3; i++) {
                ASSERT_NEAR(kv(i),ev(i),ABS_ERROR);
            }
        }

        //  void assertEqualRotations(const KDL::Rotation krot, const Eigen::Matrix3d erot) {
        //
        //      Eigen::Vector4d tmpQuat;
        //      krot.GetQuaternion(tmpQuat(0), tmpQuat(1), tmpQuat(2), tmpQuat(3));
        //      ASSERT_NEAR(tmpQuat(0), erot.x(), ABS_ERROR);
        //      ASSERT_NEAR(tmpQuat(1), erot.y(), ABS_ERROR);
        //      ASSERT_NEAR(tmpQuat(2), erot.z(), ABS_ERROR);
        //      ASSERT_NEAR(tmpQuat(3), erot.w(), ABS_ERROR);
        //  }

        /**
         This method compares a KDL Frame against an Eigen LGSM Frame by first
         transforming the LGSM Frame into a KDL Frame and using KDL::Frame to
         verify that both transformations are equivalent.

         Eigen::Displacementd => KDL::Frame
         KDL::Frame Vs. KDL::Frame

         @param kdl_frame KDL Frame
         @param eigen_frame Eigen LGSM Frame (Displacementd object)
         */
        void assertEqualFrames1(const KDL::Frame &kdl_frame, const Eigen::Displacementd &eigen_frame) {
            // Eigen::Displacementd => KDL::Frame
            Eigen::Matrix4d eigenH(4,4);
            eigenH = eigen_frame.toHomogeneousMatrix();
            KDL::Frame kdlH;
            kdlH.Make4x4(eigenH.data());
            // Now compare two frames with KDL::Frame::Equal()
            EXPECT_TRUE(KDL::Equal(kdl_frame, kdlH));
            assertEqualVectors(kdl_frame.p, eigen_frame.getTranslation());
        }


        /**
         This method compares a KDL Frame against an Eigen LGSM Frame by comparing their
         quaternion representations and position vectors independently.

         @param kdl_frame KDL Frame
         @param eigen_frame Eigen LGSM Frame (Displacementd object)
         */
        void assertEqualFrames2(const KDL::Frame &kdl_frame, const Eigen::Displacementd &eigen_frame) {
            // KDL::Frame => Eigen::Quaternion
            Eigen::Vector4d tmpQuat; // x y z w
            kdl_frame.M.GetQuaternion(tmpQuat(0), tmpQuat(1), tmpQuat(2), tmpQuat(3));

            // Compare quaternion components of the two representations
            EXPECT_NEAR(tmpQuat(0), eigen_frame.qx(), ABS_ERROR) << "x differs in quaternion comparisons between two frames";
            EXPECT_NEAR(tmpQuat(1), eigen_frame.qy(), ABS_ERROR) << "y differs in quaternion comparisons between two frames";
            EXPECT_NEAR(tmpQuat(2), eigen_frame.qz(), ABS_ERROR) << "z differs in quaternion comparisons between two frames";
            EXPECT_NEAR(tmpQuat(3), eigen_frame.qw(), ABS_ERROR) << "w differs in quaternion comparisons between two frames";

            // Compare position
            assertEqualVectors(kdl_frame.p, eigen_frame.getTranslation());
        }


        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).
        }

    };

    // Testing that the four vectors created in KDL and Eigen have the same contents.
    TEST_F(KDLvsLGSMTest, assertEqualVectors) {
        assertEqualVectors(kv1,ev1);
        assertEqualVectors(kv2,ev2);
        assertEqualVectors(kv3,ev3);
        assertEqualVectors(kv4,ev4);
    }

    TEST_F(KDLvsLGSMTest, doubleVectorOperators) {
        assertEqualVectors(2*kv2, 2*ev2);
        assertEqualVectors(kv1*2, ev1*2);
        assertEqualVectors(kv1/2, ev1/2);
    }

    // Test of vector vector operators, such as addition, substraction,
    // cross and dot product, sign inversion, equality, norm and normalization
    // setting vector to zero
    TEST_F(KDLvsLGSMTest, vectorVectorOperators) {

        // Addition and substraction
        assertEqualVectors(kv1 + kv2, ev1 + ev2);
        assertEqualVectors(kv1 - kv2, ev1 - ev2);

        // Operator += and -=
        assertEqualVectors(kv1+=kv2, ev1+=ev2);
        assertEqualVectors(kv1-=kv2, ev1-=ev2);

        // Cross and dot product
        assertEqualVectors(kv1*kv2, ev1.cross(ev2));
        ASSERT_EQ(dot(kv1,kv2), ev1.dot(ev2));

        // Sign inversion
        assertEqualVectors(-kv1, -ev1);
        KDL::Vector tmp = kv1;
        tmp.ReverseSign();
        assertEqualVectors(tmp, -ev1);

        // Norm and normalization
        tmp.Zero();
        tmp = kv3;
        tmp.Normalize(); // KDL normalizes the original vector but the
        // return is the actual norm, while Eigen does
        // not change the original vector
        assertEqualVectors(tmp, ev3.normalized());
        ASSERT_EQ(tmp.Norm(), ev3.normalized().norm());

        // Setting your vector to zero
        SetToZero(kv1);
        assertEqualVectors(kv1, ev1.setZero());
    }

    TEST_F(KDLvsLGSMTest, frames) {
        // Creating frames
        //    std::cout << "No rotation, zero trans\n" << std::endl;
        efr1 = Eigen::Displacementd(0,0,0,1,0,0,0);
        kfr1 = KDL::Frame::Identity();
        //TODO: Put more test cases.

        //    std::cout << "[LGSM]: \n" << efr1.toHomogeneousMatrix() << std::endl;
        //    std::cout << "[KDL] - last row is position: \n" << kfr1 << std::endl;
        assertEqualFrames1(kfr1, efr1);
        assertEqualFrames2(kfr1,efr1);
        
        // Frame composition
        
    }
    

} // namespace
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
