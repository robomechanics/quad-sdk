//
// Copyright (c) 2022 INRIA
//
/**
 * @file archive.hpp
 */

#ifndef PROXSUITE_SERIALIZATION_ARCHIVE_HPP
#define PROXSUITE_SERIALIZATION_ARCHIVE_HPP

#include <fstream>
#include <string>

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

namespace proxsuite {
namespace serialization {

///
/// \brief Loads an object from a std::stringstream.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[out] object Object in which the loaded data are copied.
/// \param[in]  is  string stream constaining the serialized content of the
/// object.
///
template<typename T>
inline void
loadFromStringStream(T& object, std::istringstream& is)
{
  cereal::JSONInputArchive ia(is);
  ia(object);
}

///
/// \brief Saves an object inside a std::stringstream.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[in]   object Object in which the loaded data are copied.
/// \param[out]  ss String stream constaining the serialized content of the
/// object.
///
template<typename T>
inline void
saveToStringStream(const T& object, std::stringstream& ss)
{
  cereal::JSONOutputArchive oa(ss);
  oa(object);
}

///
/// \brief Loads an object from a std::string
///
/// \tparam T Type of the object to deserialize.
///
/// \param[out] object Object in which the loaded data are copied.
/// \param[in]  str  string constaining the serialized content of the object.
///
template<typename T>
inline void
loadFromString(T& object, const std::string& str)
{
  std::istringstream is(str);
  loadFromStringStream(object, is);
}

///
/// \brief Saves an object inside a std::string
///
/// \tparam T Type of the object to deserialize.
///
/// \param[in] object Object in which the loaded data are copied.
///
/// \returns a string  constaining the serialized content of the object.
///
template<typename T>
inline std::string
saveToString(const T& object)
{
  std::stringstream ss;
  saveToStringStream(object, ss);
  return ss.str();
}

///
/// \brief Loads an object from a binary file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[out] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
inline void
loadFromBinary(T& object, const std::string& filename)
{
  std::ifstream ifs(filename.c_str(), std::ios::binary);
  if (ifs) {
    cereal::BinaryInputArchive ia(ifs);
    ia(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

///
/// \brief Saves an object inside a binary file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[in] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
void
saveToBinary(const T& object, const std::string& filename)
{
  std::ofstream ofs(filename.c_str(), std::ios::binary);
  if (ofs) {
    cereal::BinaryOutputArchive oa(ofs);
    oa(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

///
/// \brief Loads an object from a JSON file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[out] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
inline void
loadFromJSON(T& object, const std::string& filename)
{
  std::ifstream ifs(filename.c_str());
  if (ifs) {
    cereal::JSONInputArchive ia(ifs);
    ia(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

///
/// \brief Saves an object inside a JSON file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[in] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
void
saveToJSON(const T& object, const std::string& filename)
{
  std::ofstream ofs(filename.c_str());
  if (ofs) {
    cereal::JSONOutputArchive oa(ofs);
    oa(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

///
/// \brief Loads an object from a XML file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[out] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
inline void
loadFromXML(T& object, const std::string& filename)
{
  std::ifstream ifs(filename.c_str());
  if (ifs) {
    cereal::XMLInputArchive ia(ifs);
    ia(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

///
/// \brief Saves an object inside a XML file.
///
/// \tparam T Type of the object to deserialize.
///
/// \param[in] object Object in which the loaded data are copied.
/// \param[in] filename Name of the file containing the serialized data.
///
template<typename T>
void
saveToXML(const T& object, const std::string& filename)
{
  std::ofstream ofs(filename.c_str());
  if (ofs) {
    cereal::XMLOutputArchive oa(ofs);
    oa(object);
  } else {
    const std::string exception_message(filename +
                                        " does not seem to be a valid file.");
    throw std::invalid_argument(exception_message);
  }
}

}
}

#endif /* end of include guard PROXSUITE_SERIALIZATION_ARCHIVE_HPP */
