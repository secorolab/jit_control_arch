#include <jit_control_arch/jit_node.hpp>

#include <dlfcn.h>
#include <ffi.h>

#include <llvm/ExecutionEngine/Orc/Mangling.h>
#include <llvm/ExecutionEngine/Orc/SymbolStringPool.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <memory>
#include <vector>
#include <string>
#include <any>


struct FuncReturn {
    std::string type;
    std::string name;
};

struct FuncArg {
    std::string type;
    std::string name;
};

struct FuncData {
    std::string name;
    std::string ir;
    std::string lib;
    FuncReturn returnSig;
    std::vector<FuncArg> args;
};

ffi_type* mapType(const std::string& t) {
    if (t == "double") return &ffi_type_double;
    if (t == "float")  return &ffi_type_float;
    if (t == "int")    return &ffi_type_sint32;
    if (t == "long")   return &ffi_type_sint64;
    if (t == "void")   return &ffi_type_void;
    if (t.back() == '*') return &ffi_type_pointer;
    throw std::runtime_error("Unsupported type: " + t);
}

LLVMJITNode::LLVMJITNode() : Node("llvm_jit_node") {
    RCLCPP_INFO(get_logger(), "LLVM JIT Node started.");
}

void LLVMJITNode::load_ir(const char *ir_code, LLJIT &JIT) {
    auto Context = std::make_unique<LLVMContext>();
    SMDiagnostic Err;
    auto Buffer = MemoryBuffer::getMemBuffer(ir_code);
    auto M = parseIR(*Buffer, Err, *Context);

    if (!M) {
      Err.print("jit_ir_string", errs());
      RCLCPP_ERROR(get_logger(), "Failed to parse IR");
      return;
    }

    // check for error after parsing
    if (Err.getMessage().size() > 0) {
        RCLCPP_ERROR(get_logger(), "Error parsing IR: %s", Err.getMessage().str().c_str());
        return;
    }

    cantFail(JIT.addIRModule(ThreadSafeModule(std::move(M), std::move(Context))));
}

void ffiCall(void* fn_ptr,
             const std::string& returnType,
             const std::vector<std::string>& argTypes,
             const std::vector<void*>& argValues,
             void* retPtr)
{
    // Prepare ffi types
    std::vector<ffi_type*> ffi_arg_types;
    for (const auto& t : argTypes) {
        ffi_arg_types.push_back(mapType(t));
    }
    ffi_type* ffi_return_type = mapType(returnType);
    
    // Prepare ffi cif
    ffi_cif cif;
    if (ffi_prep_cif(&cif, FFI_DEFAULT_ABI, ffi_arg_types.size(),
                     ffi_return_type, ffi_arg_types.data()) != FFI_OK) {
        throw std::runtime_error("ffi_prep_cif failed");
    }
    
    // Call the function
    ffi_call(&cif, FFI_FN(fn_ptr), retPtr, const_cast<void**>(argValues.data()));
}

void LLVMJITNode::testDL() {

    struct DLFuncData {
        std::string fName;
        std::string lib;
        std::string returnType;
        std::vector<std::string> argTypes;
        std::vector<void*> argValues;
    };

    DLFuncData dl_func = {
        "p_controller",
        "libcontrollers.so",
        "double",
        {"double", "double"},
        {}
    };
    double p_gain = 2.0;
    double error = 1.5;
    dl_func.argValues.push_back(&p_gain);
    dl_func.argValues.push_back(&error);

    // Load external controllers library
    const auto pkg = "jit_control_arch";
    const auto prefix = ament_index_cpp::get_package_prefix(pkg) + "/lib/";
    const auto lib_controllers_path = prefix + dl_func.lib;

    void* handle = dlopen(lib_controllers_path.c_str(), RTLD_LAZY);
    if (!handle) {
        RCLCPP_ERROR(get_logger(), "Failed to load library: %s", dlerror());
        return;
    }
    RCLCPP_INFO(get_logger(), "Loaded library: %s", lib_controllers_path.c_str());
    dlerror(); // Clear any existing error

    // Locate function
    void* fn_ptr = dlsym(handle, dl_func.fName.c_str());
    if (!fn_ptr) {
        RCLCPP_ERROR(get_logger(), "Failed to locate function %s: %s", dl_func.fName.c_str(), dlerror());
        dlclose(handle);
        return;
    }
    RCLCPP_INFO(get_logger(), "Located function: %s at %p", dl_func.fName.c_str(), fn_ptr);

    auto returnType = mapType(dl_func.returnType);
    // allocate space for result based on return type
    std::vector<uint8_t> retBuf(returnType->size);
    void* retPtr = returnType->size > 0 ? retBuf.data() : nullptr;

    // Call function
    try {
        ffiCall(
            fn_ptr,
            dl_func.returnType,
            dl_func.argTypes,
            dl_func.argValues,
            retPtr
        );

    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Error calling function %s: %s", dl_func.fName.c_str(), e.what());
    }
    if (dl_func.returnType == "double") {
        double result;
        memcpy(&result, retPtr, sizeof(double));
        RCLCPP_INFO(get_logger(), "Function %s output: %f", dl_func.fName.c_str(), result);
    }
}

void LLVMJITNode::testJIT() {
    const char *p_c_ir = R"(
        declare double @p_controller(double, double)

        define double @pid_controller(
            double %p_gain,
            double %i_gain,
            double %d_gain,
            double %error,
            double* %integral_state,
            double* %prev_error,
            double %dt
        ) {
        entry:
            %rintegral = load double, double* %integral_state, align 8
            %rprev_error = load double, double* %prev_error, align 8

            %p_term = fmul double %p_gain, %error

            %error_mul_dt = fmul double %error, %dt
            %new_integral = fadd double %rintegral, %error_mul_dt
            store double %new_integral, double* %integral_state, align 8

            %error_diff = fsub double %error, %rprev_error
            %derror_div_dt = fdiv double %error_diff, %dt
            %d_term = fmul double %d_gain, %derror_div_dt
            store double %error, double* %prev_error, align 8

            %i_term = fmul double %i_gain, %new_integral
            %output = fadd double %p_term, %i_term
            %output_final = fadd double %output, %d_term
            ret double %output_final
        }

        define void @schedule(double* %res_ptr) {
        entry:
            ; constants must be created with fadd, not bare literals
            %kp = fadd double 0.0, 2.0
            %ki = fadd double 0.0, 0.5
            %kd = fadd double 0.0, 0.1
            %error = fadd double 0.0, 1.5
            %dt = fadd double 0.0, 0.1

            %integral_state = alloca double, align 8
            store double 0.0, double* %integral_state, align 8
            %prev_error = alloca double, align 8
            store double 0.0, double* %prev_error, align 8

            ; call p controller
            %p_res = call double @p_controller(double %kp, double %error)

            ; call pid controller
            %pid_res = call double @pid_controller(
                double %kp,
                double %ki,
                double %kd,
                double %p_res,
                double* %integral_state,
                double* %prev_error,
                double %dt
            )
            store double %pid_res, double* %res_ptr, align 8
            ret void
        }
    )";

    struct JitFuncData {
        std::string ir;
        std::string fName;
        std::vector<std::string> dlibs;
    };

    JitFuncData jit_func = {
        p_c_ir,
        "schedule",
        {"libcontrollers.so"}
    };

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    auto JIT = cantFail(LLJITBuilder().create());
    
    auto pkg = "jit_control_arch";
    auto prefix = ament_index_cpp::get_package_prefix(pkg) + "/lib/";
    
    auto &JD = JIT->getMainJITDylib();
    for (const auto &dlib : jit_func.dlibs) {
        const auto lib_path = prefix + dlib;
        auto DLGen = llvm::orc::DynamicLibrarySearchGenerator::Load(
                lib_path.c_str(),
                JIT->getDataLayout().getGlobalPrefix());
        if (!DLGen) {
            RCLCPP_ERROR(get_logger(), "Failed to load library: %s", lib_path.c_str());
            return;
        }
        JD.addGenerator(std::move(*DLGen));
        RCLCPP_INFO(get_logger(), "Loaded library: %s", lib_path.c_str());
    }

    // Load IR
    load_ir(jit_func.ir.c_str(), *JIT);
    RCLCPP_INFO(get_logger(), "Loaded JIT IR.");

    // Lookup function
    auto SymOrErr = JIT->lookup(jit_func.fName);
    if (!SymOrErr) {
        std::string msg;
        llvm::handleAllErrors(SymOrErr.takeError(), [&](llvm::ErrorInfoBase &EIB) {
            msg = EIB.message();
        });
        RCLCPP_ERROR(get_logger(), "Symbol lookup failed: %s", msg.c_str());
        return;
    }
    auto fn_ptr = SymOrErr->toPtr<void*>();
    RCLCPP_INFO(get_logger(), "Located function: %s at %p", jit_func.fName.c_str(), fn_ptr);

    using FnType = void (*)(double*);
    auto schedule_fn = reinterpret_cast<FnType>(fn_ptr);

    double result = 0.0;
    schedule_fn(&result);

    RCLCPP_INFO(get_logger(), "JIT function %s output: %f", jit_func.fName.c_str(), result);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LLVMJITNode>();
    RCLCPP_INFO(node->get_logger(), "-------> Starting DL test...");
    node->testDL();
    RCLCPP_INFO(node->get_logger(), "-------> Starting JIT test...");
    node->testJIT();
    rclcpp::shutdown();
    return 0;
}

