// This file is autogenerated. DO NOT EDIT

#pragma once
#include <robotpy_build.h>

#include <wpi/DataLog.h>

#include <pybind11/stl.h>
#include <wpi_span_type_caster.h>
#include <wpystruct.h>

namespace rpygen {

using namespace wpi::log;

template <typename T, typename I>
struct bind_wpi__log__StructLogEntry {
  py::class_<typename wpi::log::StructLogEntry<T, I>, wpi::log::DataLogEntry> cls_StructLogEntry;

  py::module &m;
  std::string clsName;
  bind_wpi__log__StructLogEntry(py::module &m, const char * clsName) :
    cls_StructLogEntry(m, clsName),
    m(m),
    clsName(clsName) {
  }

  void finish(const char * set_doc = NULL, const char * add_doc = NULL) {
    cls_StructLogEntry.doc() = "Log raw struct serializable objects.";
    cls_StructLogEntry
      .def(py::init([](DataLog &log, std::string_view name, const py::type &t, int64_t timestamp) {
        WPyStructInfo info(t);
        return std::make_shared<StructLogEntry<WPyStruct, WPyStructInfo>>(log, name, info, timestamp);
      }
      )
        , py::arg("log"), py::arg("name"), py::arg("type"), py::arg("timestamp") = 0
        , py::keep_alive<1, 2>()
      )
      .def(py::init([](DataLog &log, std::string_view name, std::string_view metadata, const py::type &t, int64_t timestamp) {
        WPyStructInfo info(t);
        return std::make_shared<StructLogEntry<WPyStruct, WPyStructInfo>>(log, name, metadata, info, timestamp);
      }
      )
        , py::arg("log"), py::arg("name"), py::arg("metadata"), py::arg("type"), py::arg("timestamp") = 0
        , py::keep_alive<1, 2>()
      )
      .def("append", &wpi::log::StructLogEntry<T, I>::Append
        , py::arg("data"), py::arg("timestamp") = 0
        , release_gil()
        , py::doc(
        "Appends a record to the log.\n"
        "\n"
        ":param data:      Data to record\n"
        ":param timestamp: Time stamp (may be 0 to indicate now)")
      )
      .def("update", &wpi::log::StructLogEntry<T, I>::Update
        , py::arg("data"), py::arg("timestamp") = 0
        , release_gil()
        , py::doc(
        "Updates the last value and appends a record to the log if it has changed.\n"
        "\n"
        ".. note:: The last value is local to this class instance; using Update() with\n"
        "   two instances pointing to the same underlying log entry name will likely\n"
        "   result in unexpected results.\n"
        "\n"
        ":param data:      Data to record\n"
        ":param timestamp: Time stamp (may be 0 to indicate now)")
      )
      .def("hasLastValue", &wpi::log::StructLogEntry<T, I>::HasLastValue
        , release_gil()
        , py::doc(
        "Gets whether there is a last value.\n"
        "\n"
        ".. note:: The last value is local to this class instance and updated only with\n"
        "   Update(), not Append().\n"
        "\n"
        ":returns: True if last value exists, false otherwise.")
      )
      .def("getLastValue", &wpi::log::StructLogEntry<T, I>::GetLastValue
        , release_gil()
        , py::doc(
        "Gets the last value.\n"
        "\n"
        ".. note:: The last value is local to this class instance and updated only with\n"
        "   Update(), not Append().\n"
        "\n"
        ":returns: Last value (empty if no last value)")
      )
    ;
    if (set_doc) {
      cls_StructLogEntry.doc() = set_doc;
    }
    if (add_doc) {
      cls_StructLogEntry.doc() = py::cast<std::string>(cls_StructLogEntry.doc()) + add_doc;
    }
  }
}; // struct bind_wpi__log__StructLogEntry

}; // namespace rpygen
