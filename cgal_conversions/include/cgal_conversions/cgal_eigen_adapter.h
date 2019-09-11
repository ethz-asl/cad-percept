#ifndef CGAL_CONVERSIONS_CGAL_EIGEN_ADAPTER_H_
#define CGAL_CONVERSIONS_CGAL_EIGEN_ADAPTER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <Eigen/Dense>
namespace cad_percept {
namespace cgal {

/*
 * This header-only library provides functionality to design functions such that they
 * automatically accept eigen and cgal vectors without templates/overloading and the like.
 * Makes Eigen::Vector3d / cgal::Vector_3 / cgal::Point_3 interchangeable.
 *
 * Examples:
 * You want a function that accepts a Vector3 (regardless of eigen/cgal):
 * Signature: void yourFunction(Vector3In vec)
 * Usage:     Whenever you assign vec to a cgal or eigen vector, it gets converted transparently.
 *            Inside the function: e.g. Eigen::Vector3d test = vec;
 *                                      cgal::Vector test2 = vec;
 *
 *            Calling the function: yourFunction(test) (where test is of type Eigen::Vector3d or
 *                                                      cgal::Vector/Point, doesn't matter)
 *
 * You want a function that returns a Vector3 that can be assigned to eigen / cgal:
 * Signature: Vector3Return yourFunction()
 * Usage:     Eigen::Vector3d test = yourFunction();
 *            cgal::Vector test2 = yourFunction();
 *
 * Inside function: return Eigen::Vector3d(...) or cgal::Vector(...)  or cgal::Point(...)
 *
 *
 * You want a function with a Vector3 as output parameter:
 * Signature: void yourFunction(Vector3Out vec_out)
 * Usage inside function: Just assign a eigen/cgal vector to vec_out
 * Calling function: yourFunction(&test), where test is of type Eigen::Vector3d or cgal::Vector or
 * cgal::Point)
 *
 */

/*
 * Abstract base class for functionality
 * Contains x,y,z data for a vector.
 * Can currently only deal with 3d, non-homogenous data. (hz = 0 in cgal)
 */
class Vector3Adap {
 public:
  Vector3Adap(double x, double y, double z) : x_(x), y_(y), z_(z) {}

 protected:
  inline Vector3Adap(cgal::Vector vec) : Vector3Adap(vec.x(), vec.y(), vec.z()) {}
  inline Vector3Adap(cgal::Point pt) : Vector3Adap(pt.x(), pt.y(), pt.z()) {}
  inline Vector3Adap(Eigen::Vector3d eigen) : Vector3Adap(eigen.x(), eigen.y(), eigen.z()) {}

  virtual void dummy() = 0;  // method to mark this class as purely abstract
  double x_, y_, z_;
};

/*
 * Class that supports automatic casting from a
 *  Eigen::Vector3 pointer,
 *  cgal::Vector pointer,
 *  cgal::Point pointer
 *
 *  and allows assignemetns from these types that are reflected back to the pointer.
 *
 */
class Vector3Out : public Vector3Adap {
 public:
  /*
   * Implicit cast-constructors for all 3 types.
   */
  inline Vector3Out(cgal::Vector* vec)
      : Vector3Adap(*vec),
        external_eigen_(nullptr),
        external_cgal_(vec),
        external_point_(nullptr) {}

  inline Vector3Out(Eigen::Vector3d* vec)
      : Vector3Adap(*vec),
        external_eigen_(vec),
        external_cgal_(nullptr),
        external_point_(nullptr) {}

  inline Vector3Out(cgal::Point* pt)
      : Vector3Adap(*pt), external_eigen_(nullptr), external_cgal_(nullptr), external_point_(pt) {}

  /*
   * Assignement operators for all 3 types.
   */
  inline void operator=(const Eigen::Vector3d& value) {
    x_ = value.x();
    y_ = value.y();
    z_ = value.z();
    sync();
  }

  inline void operator=(const cgal::Vector& value) {
    x_ = value.x();
    y_ = value.y();
    z_ = value.z();
    sync();
  }

  inline void operator=(const cgal::Point& value) {
    x_ = value.x();
    y_ = value.y();
    z_ = value.z();
    sync();
  }

  inline void dummy() final {}
  inline ~Vector3Out() {
    // write data to pointers before becoming out of scope.
    sync();
  }

 private:
  inline void sync() {
    if (external_eigen_ != nullptr) {
      external_eigen_->x() = x_;
      external_eigen_->y() = y_;
      external_eigen_->z() = z_;
    }

    if (external_cgal_ != nullptr) {
      *external_cgal_ = cgal::Vector(x_, y_, z_);
    }
    if (external_point_ != nullptr) {
      *external_point_ = cgal::Point(x_, y_, z_);
    }
  }

  // Pointers to external data.
  Eigen::Vector3d* external_eigen_;
  cgal::Vector* external_cgal_;
  cgal::Point* external_point_;
};

/*
 * Class that supports automatic casting from a
 *  Eigen::Vector3, cgal::Vector, cgal::Point
 *
 * and supports casting to any 3 of these.
 */
class Vector3In : public Vector3Adap {
 public:
  inline Vector3In(cgal::Vector vec) : Vector3Adap(vec) {}
  inline Vector3In(Eigen::Vector3d eigen) : Vector3Adap(eigen) {}
  inline Vector3In(cgal::Point pt) : Vector3Adap(pt) {}

  inline operator cgal::Vector() { return {x_, y_, z_}; }
  inline operator Eigen::Vector3d() { return {x_, y_, z_}; }
  inline operator cgal::Point() { return {x_, y_, z_}; }

  inline void dummy() final {}
};

typedef Vector3In Vector3Return;

}  // namespace cgal
}  // namespace cad_percept

#endif  // CGAL_CONVERSIONS_CGAL_EIGEN_ADAPTER_H_
