// #include "this/package/foo.h"
#include "gtest/gtest.h"
#include "kdl/frames.hpp"
#include "kdl/frames_io.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/Lgsm"
#include <array>
#include <iostream>
#include <cmath>


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

        // KDL Twists
        KDL::Twist kt1;

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

            kt1 = KDL::Twist::Zero();
            kt1(0) = 1.0;

        }

        // Compares (asserts) two equal vectors. KDL vs Eigen
        void assertEqual3dVectors(KDL::Vector kv, Eigen::Vector3d ev) {
            for (size_t i = 0; i < 3; i++) {
                ASSERT_NEAR(kv(i),ev(i),ABS_ERROR);
            }
        }

        void assertEqualKDLTwists(KDL::Twist &kt1, KDL::Twist &kt2) {
            for (size_t i = 0; i < 6; i++) {
                ASSERT_EQ(kt1(i), kt2(i));
            }
        }
        
        
        /**
         Compares two vectors, for the moment they should be of the same size.

         @param eigVec1 First vector.
         @param eigVec2 Second vector.
         */
        template <typename Derived1, typename Derived2>
        void assertEqualEigenVectors(const Eigen::MatrixBase<Derived1>& eigVec1, const Eigen::MatrixBase<Derived2>& eigVec2)
        {
            for (int i=0;i<3;i++) {
                ASSERT_NEAR(eigVec1(i),eigVec2(i), ABS_ERROR);
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
            assertEqual3dVectors(kdl_frame.p, eigen_frame.getTranslation());
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
            assertEqual3dVectors(kdl_frame.p, eigen_frame.getTranslation());
        }

        /**
         Returns pointer to data stored in KDL Twist. Well, actually to the copy of this data.

         @param kdlTwist Input KDL Twist.
         @return Pointer to data.
         */
        double* getTwistData(const KDL::Twist &kdlTwist) {
            std::vector<double> outTwist(3);
            outTwist.assign(&kdlTwist.vel.data[0], &kdlTwist.vel.data[0] + 3);
            outTwist.insert(outTwist.end(), &kdlTwist.rot.data[0], &kdlTwist.rot.data[0] + 3);
            return outTwist.data();
        }


        virtual void TearDown() {
            // Code here will be called immediately after each test (right
            // before the destructor).
        }

    };


    /**
     Addition between an Eigen Vector and a KDL Twist

     @param eigVector Eigen Vector. Should be of size 6x1.
     @param kdlTwist KDL Twist.
     @return Resulting addition of the two twists.
     */
    template <typename Derived>
    KDL::Twist operator+(const Eigen::DenseBase<Derived> &eigVector, const KDL::Twist &kdlTwist) {
        EXPECT_EQ(eigVector.rows(), 6);
        KDL::Twist kdlTwist1;
        kdlTwist1(0) = kdlTwist(0) + eigVector(0);
        kdlTwist1(1) = kdlTwist(1) + eigVector(1);
        kdlTwist1(2) = kdlTwist(2) + eigVector(2);
        kdlTwist1(3) = kdlTwist(3) + eigVector(3);
        kdlTwist1(4) = kdlTwist(4) + eigVector(4);
        kdlTwist1(5) = kdlTwist(5) + eigVector(5);
        return kdlTwist1;
    }


    /**
     Multiplication operator between Eigen Matrices and KDL Twists.

     @param eigMatrix Eigen Matrix. Must have 6 columns.
     @param kdlTwist KDL Twist.
     @return Result of the multiplication as a KDL Twist.
     */
    template <typename Derived>
    KDL::Twist operator*(const Eigen::MatrixBase<Derived> &eigMatrix, const KDL::Twist &kdlTwist) {
        EXPECT_EQ(eigMatrix.cols(), 6);
        Eigen::VectorXd tmpEig(6);
        tmpEig(0) = kdlTwist(0);
        tmpEig(1) = kdlTwist(1);
        tmpEig(2) = kdlTwist(2);
        tmpEig(3) = kdlTwist(3);
        tmpEig(4) = kdlTwist(4);
        tmpEig(5) = kdlTwist(5);

        Eigen::VectorXd res(6);
        res = eigMatrix * tmpEig;
        KDL::Twist out = KDL::Twist(KDL::Vector(res(0), res(1), res(2)), KDL::Vector(res(3), res(4), res(5)));
        return out;
    }
    
    
    /**
     Implements the capitalized logarithmic map, which directly provides the anlge phi and axis u of rotation in cartesian 3D space, in the form u*phi. It returns only the vector coefficients of the resulting quaternion (to comply with EigenLGSM).

     @param[in] q Input quaternion. Use a fixed-sized Eigen vector, e.g. Eigen::Vector4d.
     @param[out] log Three-dimensional logarithm of the input quaternion. Use a fixed-size Eigen vector, e.g. Eigen::Vector3d.
     */
    template <typename Derived1, typename Derived2>
    void quaternionLog(const Eigen::MatrixBase<Derived1>& q, Eigen::MatrixBase<Derived2>& log) {
        Eigen::Vector4d tmp;
        // As done in http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
        // Page 21: The logarithmic maps
        double qv_norm = q.head(3).norm();
        double qw = q(3);
        double phi = 2*std::atan2(qv_norm, qw);
        tmp = (phi/qv_norm)*q;
        log = tmp.head(3);
    }

    // Testing that the four vectors created in KDL and Eigen have the same contents.
    TEST_F(KDLvsLGSMTest, assertEqual3dVectors) {
        assertEqual3dVectors(kv1,ev1);
        assertEqual3dVectors(kv2,ev2);
        assertEqual3dVectors(kv3,ev3);
        assertEqual3dVectors(kv4,ev4);
    }

    TEST_F(KDLvsLGSMTest, doubleVectorOperators) {
        assertEqual3dVectors(2*kv2, 2*ev2);
        assertEqual3dVectors(kv1*2, ev1*2);
        assertEqual3dVectors(kv1/2, ev1/2);
    }

    // Test of vector vector operators, such as addition, substraction,
    // cross and dot product, sign inversion, equality, norm and normalization
    // setting vector to zero
    TEST_F(KDLvsLGSMTest, vectorVectorOperators) {

        // Addition and substraction
        assertEqual3dVectors(kv1 + kv2, ev1 + ev2);
        assertEqual3dVectors(kv1 - kv2, ev1 - ev2);

        // Operator += and -=
        assertEqual3dVectors(kv1+=kv2, ev1+=ev2);
        assertEqual3dVectors(kv1-=kv2, ev1-=ev2);

        // Cross and dot product
        assertEqual3dVectors(kv1*kv2, ev1.cross(ev2));
        ASSERT_EQ(dot(kv1,kv2), ev1.dot(ev2));

        // Sign inversion
        assertEqual3dVectors(-kv1, -ev1);
        KDL::Vector tmp = kv1;
        tmp.ReverseSign();
        assertEqual3dVectors(tmp, -ev1);

        // Norm and normalization
        tmp.Zero();
        tmp = kv3;
        tmp.Normalize(); // KDL normalizes the original vector but the
        // return is the actual norm, while Eigen does
        // not change the original vector
        assertEqual3dVectors(tmp, ev3.normalized());
        ASSERT_EQ(tmp.Norm(), ev3.normalized().norm());

        // Setting your vector to zero
        SetToZero(kv1);
        assertEqual3dVectors(kv1, ev1.setZero());
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

    TEST_F(KDLvsLGSMTest, framesComposition) {
        // F_A_C = F_A_B * F_B_C;
        // First with KDL
        KDL::Frame F_A_B(KDL::Rotation::RPY(0, M_PI/2, 0));
        KDL::Frame F_B_C(KDL::Rotation::RPY(M_PI/4, 0, 0));
        KDL::Frame F_A_C = F_A_B * F_B_C;

        // Then with LGSM

    }

    TEST_F(KDLvsLGSMTest, additionOperator) {
        // First create an Eigen Twist equal to kt1
        Eigen::VectorXd et1(6);
        et1.setZero();
        et1(0) = 1.0;
        // Add KDL Twist and Eigen Vector
        KDL::Twist sumResult = et1 + kt1;
        // The sum of the first two components should be 2
        ASSERT_EQ(sumResult(0), 2);

        // Testing with the subcolumn of 6-dimensional matrix
        Eigen::MatrixXd mat(6,2);
        mat.setZero();
        mat(0,0) = 1;
        sumResult = mat.leftCols(1) + kt1;
        ASSERT_EQ(sumResult(0),2);

        // Testing with the result of a multiplication between an Eigen matrix and an Eigen Vector
        Eigen::Vector2d tmp; tmp << 1, 0;
        sumResult = mat*tmp + kt1;
        ASSERT_EQ(sumResult(0),2);
    }

    TEST_F(KDLvsLGSMTest, multiplicationOperator) {
        // First create 6x6 Eigen Matrix
        Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(6, 6);
        // Multiply by member KDL twist
        KDL::Twist res = mat*kt1;
        assertEqualKDLTwists(kt1, res);
        // Testing multiplication with a block
        KDL::Twist tmp = mat.leftCols(6)*kt1;
        assertEqualKDLTwists(kt1, tmp);
    }

    TEST_F(KDLvsLGSMTest, twistData) {
        double* twistData = getTwistData(kt1);
//        for (int i=0; i<=6; i++)
//            std::cout << twistData[i] << std::endl;
        EXPECT_EQ(twistData[0], kt1(0));
    }

    TEST_F(KDLvsLGSMTest, Rotations3D) {
        Eigen::MatrixXd H_eig(4,4); H_eig.setZero();
        Eigen::VectorXd H(16);
        KDL::Frame H_kdl;
        // Rotate around x by 45 degrees
        H_kdl.M.DoRotX(45);
        H_kdl.Make4x4(H.data());
//        std::cout << "H_kdl: \n" << H_kdl << std::endl;
        H_eig = Eigen::Map<Eigen::MatrixXd>(H.data(),4,4);
        H_eig.transposeInPlace();
//        std::cout << "Corresponding Eigen Matrix\n" << H_eig << std::endl;
        //TODO: Properly compare the KDL frame and the corresponding homogeneous matrix.
    }
    
    TEST_F(KDLvsLGSMTest, quaternionLog) {
        KDL::Frame H_kdl;
        // Rotation around x by 45 degrees
        H_kdl.M.DoRotX(45);
        H_kdl.M.DoRotY(30);
        // Get the equivalent quaternion in this order [x y z w]
        Eigen::Vector4d quat;
        H_kdl.M.GetQuaternion(quat(0), quat(1), quat(2), quat(3));
        Eigen::Vector3d log;
        quaternionLog(quat, log);
        std::cout << "My Log: " << log.transpose() << std::endl;
        // Build the same quaternion with LGSM
        Eigen::Displacementd::Rotation3D quat_lgsm;
        quat_lgsm.x() = quat(0);
        quat_lgsm.y() = quat(1);
        quat_lgsm.z() = quat(2);
        quat_lgsm.w() = quat(3);
        std::cout << "LGSM Log: " << quat_lgsm.log().get().transpose() << std::endl;
        Eigen::Vector3d tmp; tmp << quat_lgsm.log().get().operator()(0),
                                       quat_lgsm.log().get().operator()(1),
                                       quat_lgsm.log().get().operator()(2);
        assertEqualEigenVectors(log, tmp);
    }
    
} // namespace
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
