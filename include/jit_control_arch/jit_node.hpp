#ifndef JIT_CONTROL_ARCH_JIT_NODE_HPP
#define JIT_CONTROL_ARCH_JIT_NODE_HPP

#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/ExecutionEngine/Orc/ThreadSafeModule.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/MemoryBuffer.h"

#include "rclcpp/rclcpp.hpp"

using namespace llvm;
using namespace llvm::orc;

class LLVMJITNode : public rclcpp::Node {
public:
  LLVMJITNode();

  void load_ir(const char *ir_code, LLJIT &JIT);

  void test();
};

#endif  // JIT_CONTROL_ARCH_JIT_NODE_HPP
