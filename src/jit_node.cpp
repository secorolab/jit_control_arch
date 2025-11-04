#include <jit_control_arch/jit_node.hpp>

LLVMJITNode::LLVMJITNode() : Node("llvm_jit_node") {
    RCLCPP_INFO(get_logger(), "LLVM JIT Node started.");
    test();
    test2();
}

void LLVMJITNode::test() {
    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();

    auto JIT = cantFail(LLJITBuilder().create());
    auto Context = std::make_unique<LLVMContext>();
    auto M = std::make_unique<Module>("test_module", *Context);

    FunctionType *FT = FunctionType::get(Type::getInt32Ty(*Context),
                                         {Type::getInt32Ty(*Context), Type::getInt32Ty(*Context)},
                                         false);
    Function *F = Function::Create(FT, Function::ExternalLinkage, "add", M.get());

    BasicBlock *BB = BasicBlock::Create(*Context, "entry", F);
    IRBuilder<> builder(BB);
    auto args = F->args();
    auto it = args.begin();
    Value *a = &(*it++);
    Value *b = &(*it++);
    Value *sum = builder.CreateAdd(a, b, "sum");
    builder.CreateRet(sum);

    cantFail(JIT->addIRModule(ThreadSafeModule(std::move(M), std::move(Context))));
    auto Sym = cantFail(JIT->lookup("add"));
    auto *addFunc = Sym.toPtr<int (*)(int, int)>();

    int result = addFunc(3, 9);
    RCLCPP_INFO(get_logger(), "JIT-compiled add(3,9) = %d", result);
}

void LLVMJITNode::test2() {
    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    auto JIT = cantFail(LLJITBuilder().create());

    const char *ir = R"(
        define i32 @add(i32 %a, i32 %b) {
        entry:
            %sum = add i32 %a, %b
            ret i32 %sum
        }
    )";

    auto Context = std::make_unique<LLVMContext>();
    SMDiagnostic Err;
    auto Buffer = MemoryBuffer::getMemBuffer(ir);
    auto M = parseIR(*Buffer, Err, *Context);

    if (!M) {
      Err.print("jit_ir_string", errs());
      RCLCPP_ERROR(get_logger(), "Failed to parse IR");
      return;
    }

    cantFail(JIT->addIRModule(ThreadSafeModule(std::move(M), std::move(Context))));
    auto Sym = cantFail(JIT->lookup("add"));
    auto *addFunc = Sym.toPtr<int (*)(int, int)>();
    int result = addFunc(5, 7);

    RCLCPP_INFO(get_logger(), "JIT-IR add(5,7) = %d", result);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LLVMJITNode>());
    rclcpp::shutdown();
    return 0;
}

