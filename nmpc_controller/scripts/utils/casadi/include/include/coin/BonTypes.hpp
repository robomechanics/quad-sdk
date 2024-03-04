#ifndef __BonTypes_H_
#define __BonTypes_H_
#include<vector>
#include "CoinSmartPtr.hpp"

namespace Bonmin {
/** A small wrap around std::vector to give easy access to array for interfacing with fortran code.*/
template<typename T>
class vector : public std::vector<T>{
public:
  /** Default constructor.*/
  vector(): std::vector<T>(){}
  /** Constructor with initialization.*/
  vector(size_t n, const T& v): std::vector<T>(n,v){}
  /** Copy constructor.*/
  vector(const vector<T>& other): std::vector<T>(other){}
  /** Copy constructor.*/
  vector(const std::vector<T>& other): std::vector<T>(other){}
  /** constructor with size.*/
  vector(size_t n): std::vector<T>(n){}
  /** Assignment.*/
  vector<T>& operator=(const vector<T>& other){
     std::vector<T>::operator=(other);
     return (*this);}
  /** Assignment.*/
  vector<T>& operator=(const std::vector<T>& other){
     return std::vector<T>::operator=(other);
     return (*this);}

/** Access pointer to first element of storage.*/
inline T* operator()(){
  return &(*std::vector<T>::begin());
}
/** Access pointer to first element of storage.*/
inline const T* operator()() const {
  return &(*std::vector<T>::begin());
}
};

//structure to store an object of class X in a Coin::ReferencedObject
template<class X>
struct SimpleReferenced : public Coin::ReferencedObject {
 /** The object.*/
 X object;

 const X& operator()() const{
   return object;}

 X& operator()() {
   return object;}

};
//structure to store a pointer to an object of class X in a 
// Coin::ReferencedObject
template<class X>
struct SimpleReferencedPtr : public Coin::ReferencedObject {
 /** The object.*/
 X * object;

 SimpleReferencedPtr():
   object(NULL){}

 ~SimpleReferencedPtr(){
   delete object;}

 const X& operator()() const{
   return *object;}

 X& operator()() {
   return *object;}

 const X* ptr() const{
    return object;}

 X* ptr(){
    return object;}
};

template <class X>
SimpleReferenced<X> * make_referenced(X other){
  SimpleReferenced<X> * ret_val = new SimpleReferenced<X>;
  ret_val->object = other;
  return ret_val;
}
template <class X>
SimpleReferencedPtr<X> * make_referenced(X* other){
  SimpleReferencedPtr <X> * ret_val = new SimpleReferencedPtr<X>;
  ret_val->object = other;
  return ret_val;
}


}
#endif

