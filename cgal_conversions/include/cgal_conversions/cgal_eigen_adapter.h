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
 * See Typedefs at end of file as well. Now it is templated for 2d and/or 3d vectors.
 * All following examples are for 3d vectors, but usage is the same with 2d vectors
 * (exchange Vector3In with Vector2In etc).
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
 * Can currently only deal with 2d and 3d, non-homogenous data. (hz = 0 in cgal)
 */

template <int N>
class VectorAdap {
  // Make sure this only builds for N=2 or N=3.
  static_assert(N == 2 || N == 3, "ONLY IMPLEMENTED FOR 2D/3D VECTOR TYPES.");

 public:
  /*
   * Conditional typedefs to always get the right cgal/eigen type for our needs / template
   * specializations.
   */
  typedef typename std::conditional<
      /*    */ (N == 2),
      /* y? */ cgal::Vector_2,
      /* n? */ cgal::Vector>::type cgalVector;

  typedef typename std::conditional<
      /*    */ (N == 2),
      /* y? */ cgal::Point_2,
      /* n? */ cgal::Point>::type cgalPoint;

  typedef typename std::conditional<
      /*    */ (N == 2),
      /* y? */ Eigen::Vector2d,
      /* n? */ Eigen::Vector3d>::type eigenVector;

 protected:
  inline VectorAdap() {}

  inline VectorAdap(cgalVector vec) : VectorAdap() { toData(vec, data_); }
  inline VectorAdap(cgalPoint pt) : VectorAdap() { toData(pt, data_); }
  inline VectorAdap(eigenVector eigen) : VectorAdap() { toData(eigen, data_); }

  /*
   * Methods that decouple the conversion from/to the internal datastructure
   * and allow template specialization as we need it here.
   */
  template <class T>
  inline static void toData(const T& vec, double* data);

  template <class T>
  inline static void fromData(const double* data, T* vec);

  virtual void dummy() = 0;  // method to mark this class as purely abstract
  double data_[N];
};

/* Template specializations for toData
 * Compiles for all T that have that same interface (which by chance is true with eigen/cgal here).
 */
template <>         // template instantiation on class level, thus empty <>
template <class T>  // normal template on function level
inline void VectorAdap<3>::toData(const T& vec, double* data) {
  data[0] = vec.x();
  data[1] = vec.y();
  data[2] = vec.z();
}

template <>
template <class T>
inline void VectorAdap<2>::toData(const T& vec, double* data) {
  data[0] = vec.x();
  data[1] = vec.y();
}

/* Template specializations for fromData */
template <>
template <class T>
inline void VectorAdap<3>::fromData(const double* data, T* vec) {
  *vec = {data[0], data[1], data[2]};
}
template <>
template <class T>
inline void VectorAdap<2>::fromData(const double* data, T* vec) {
  *vec = {data[0], data[1]};
}

/*
 * Class that supports automatic casting from a
 *  Eigen::Vector3 pointer,
 *  cgal::Vector pointer,
 *  cgal::Point pointer
 *
 *  and allows assignments from these types that are reflected back to the pointer.
 *
 */
template <int N>
class VectorOut : public VectorAdap<N> {
 public:
  /*
   * Implicit cast-constructors for all 3 types.
   */
  inline VectorOut(typename VectorAdap<N>::cgalVector* vec)
      : VectorAdap<N>(*vec),
        external_eigen_(nullptr),
        external_cgal_(vec),
        external_point_(nullptr) {}

  inline VectorOut(typename VectorAdap<N>::eigenVector* vec)
      : VectorAdap<N>(*vec),
        external_eigen_(vec),
        external_cgal_(nullptr),
        external_point_(nullptr) {}

  inline VectorOut(typename VectorAdap<N>::cgalPoint* pt)
      : VectorAdap<N>(*pt),
        external_eigen_(nullptr),
        external_cgal_(nullptr),
        external_point_(pt) {}

  // Disable assignment of Vector3Out itself
  void operator=(const VectorOut&) = delete;

  /*
   * Assignment operators for all 3 types.
   */
  inline void operator=(const typename VectorAdap<N>::eigenVector& value) {
    this->toData(value, this->data_);
    sync();
  }
  inline void operator=(const typename VectorAdap<N>::cgalVector& value) {
    this->toData(value, this->data_);
    sync();
  }

  inline void operator=(const typename VectorAdap<N>::cgalPoint& value) {
    this->toData(value, this->data_);
    sync();
  }

  inline void dummy() final {}
  inline ~VectorOut() {
    // write data to pointers before becoming out of scope.
    sync();
  }

 private:
  inline void sync() {
    if (external_eigen_ != nullptr) {
      this->fromData(this->data_, external_eigen_);
    }

    if (external_cgal_ != nullptr) {
      this->fromData(this->data_, external_cgal_);
    }
    if (external_point_ != nullptr) {
      this->fromData(this->data_, external_point_);
    }
  }

  // Pointers to external data.
  typename VectorAdap<N>::eigenVector* external_eigen_;
  typename VectorAdap<N>::cgalVector* external_cgal_;
  typename VectorAdap<N>::cgalPoint* external_point_;
};

/*
 * Class that supports automatic casting from a
 *  Eigen::Vector3, cgal::Vector, cgal::Point
 *
 * and supports casting to any 3 of these.
 */
template <int N>
class VectorIn : public VectorAdap<N> {
 public:
  inline VectorIn(cgal::Vector vec) : VectorAdap<N>(vec) {}
  inline VectorIn(Eigen::Vector3d eigen) : VectorAdap<N>(eigen) {}
  inline VectorIn(cgal::Point pt) : VectorAdap<N>(pt) {}

  // Disable assignment of Vector3In itself
  void operator=(const VectorIn&) = delete;

  // Casts to eigen/cgal.
  inline operator cgal::Vector() {
    cgal::Vector temp;
    this->fromData(this->data_, &temp);
    return temp;
  }

  inline operator Eigen::Vector3d() {
    Eigen::Vector3d temp;
    this->fromData(this->data_, &temp);
    return temp;
  }
  inline operator cgal::Point() {
    cgal::Point temp;
    this->fromData(this->data_, &temp);
    return temp;
  }

 private:
  inline void dummy() final {}
};

// Typedefs for 2/3d
template <int N>
using VectorReturn = VectorIn<N>;
typedef VectorOut<2> Vector2Out;
typedef VectorIn<2> Vector2In;
typedef VectorReturn<2> Vector2Return;

typedef VectorOut<3> Vector3Out;
typedef VectorIn<3> Vector3In;
typedef VectorReturn<3> Vector3Return;

}  // namespace cgal
}  // namespace cad_percept

#endif  // CGAL_CONVERSIONS_CGAL_EIGEN_ADAPTER_H_
