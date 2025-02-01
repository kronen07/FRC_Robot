// This file is autogenerated. DO NOT EDIT
#include <robotpy_build.h>

#include <wpi/DataLog.h>

#include <pybind11/stl.h>
#include <wpi_span_type_caster.h>
#include <wpystruct.h>

#define RPYGEN_ENABLE_wpi__log__DataLog_PROTECTED_CONSTRUCTORS
#include <rpygen/wpi__log__DataLog.hpp>
#include <rpygen/wpi__log__DataLogValueEntryImpl.hpp>
#include "DataLog_tmpl.hpp"

namespace rpygen {

using BindType = rpygen::bind_wpi__log__DataLogValueEntryImpl<bool>;
static std::unique_ptr<BindType> inst;

bind_wpi__log__DataLogValueEntryImpl_3::bind_wpi__log__DataLogValueEntryImpl_3(py::module &m, const char * clsName)
{
  inst = std::make_unique<BindType>(m, clsName);
}

void bind_wpi__log__DataLogValueEntryImpl_3::finish(const char *set_doc, const char *add_doc)
{
  inst->finish(set_doc, add_doc);
  inst.reset();
}

}; // namespace rpygen

