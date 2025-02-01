// This file is autogenerated. DO NOT EDIT

#pragma once
#include <robotpy_build.h>

#include <rev/config/SignalsConfig.h>

#include <rpygen/rev__BaseConfig.hpp>

namespace rev::spark {

template <typename CfgBase = rpygen::EmptyTrampolineCfg>
struct PyTrampolineCfg_SignalsConfig :
  rev::PyTrampolineCfg_BaseConfig<
  CfgBase
  >
{
  using Base = rev::spark::SignalsConfig;
};

template <typename PyTrampolineBase, typename PyTrampolineCfg>
using PyTrampolineBase_SignalsConfig =
  rev::PyTrampoline_BaseConfig<
    PyTrampolineBase
    , PyTrampolineCfg>
;

template <typename PyTrampolineBase, typename PyTrampolineCfg>
struct PyTrampoline_SignalsConfig : PyTrampolineBase_SignalsConfig<PyTrampolineBase, PyTrampolineCfg> {
  using PyTrampolineBase_SignalsConfig<PyTrampolineBase, PyTrampolineCfg>::PyTrampolineBase_SignalsConfig;
};

}; // namespace rev::spark
