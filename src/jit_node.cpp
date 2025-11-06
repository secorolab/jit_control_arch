#include <jit_control_arch/jit_node.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <memory>
#include <vector>
#include <string>
#include <ffi.h>
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

std::any callFunction(void* fn_ptr,
                    const FuncReturn& retSig,
                    std::vector<FuncArg> funcArgs,
                    std::vector<void*> args) 
{
    ffi_cif cif;
    std::vector<ffi_type*> arg_ffi;
    for (auto& t : funcArgs)
        arg_ffi.push_back(mapType(t.type));

    ffi_type* ret_type = mapType(retSig.type);
    if (ffi_prep_cif(&cif, FFI_DEFAULT_ABI,
                     arg_ffi.size(), ret_type, arg_ffi.data()) != FFI_OK)
        throw std::runtime_error("ffi_prep_cif failed");

    if (retSig.type == "double") {
        double result;
        ffi_call(&cif, FFI_FN(fn_ptr), &result, args.data());
        return result;
    } else if (retSig.type == "float") {
        float result;
        ffi_call(&cif, FFI_FN(fn_ptr), &result, args.data());
        return result;
    } else if (retSig.type == "int") {
        int result;
        ffi_call(&cif, FFI_FN(fn_ptr), &result, args.data());
        return result;
    } else if (retSig.type == "long") {
        long result;
        ffi_call(&cif, FFI_FN(fn_ptr), &result, args.data());
        return result;
    } else if (retSig.type == "void") {
        ffi_call(&cif, FFI_FN(fn_ptr), nullptr, args.data());
        return {};
    } else {
        throw std::runtime_error("Unsupported return type: " + retSig.type);
    }
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

    cantFail(JIT.addIRModule(ThreadSafeModule(std::move(M), std::move(Context))));
}

void LLVMJITNode::test() {
    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    auto JIT = cantFail(LLJITBuilder().create());
    
    // Allow the JIT to resolve symbols from the host process
    JIT->getMainJITDylib().addGenerator(
        cantFail(llvm::orc::DynamicLibrarySearchGenerator::GetForCurrentProcess(
            JIT->getDataLayout().getGlobalPrefix())));
    
    // variables IR string
    const char *variables_c_ir = R"(
        @p_gain = global double 2.0
        @i_gain = global double 0.5
        @d_gain = global double 0.1
        @error = global double 1.5
        @integral_state = global double 0.0
        @prev_error = global double 0.0
        @dt = global double 0.1
    )";

    // controllers IR strings
    const char *p_c_ir = R"(
        declare double @p_controller(double, double)

        define double @run_p(double %p_gain, double %error) {
        entry:
            %res = call double @p_controller(double %p_gain, double %error)
            ret double %res
        }
    )";

    const char *pi_c_ir = R"(
        declare double @pi_controller(double, double, double, double*, double)

        define double @run_pi(double %p_gain, double %i_gain, double %error, double* %integral_state, double %dt) {
        entry:
            %res = call double @pi_controller(double %p_gain, double %i_gain, double %error, double* %integral_state, double %dt)
            ret double %res
        }
    )";

    const char *pid_c_ir = R"(
        declare double @pid_controller(double, double, double, double, double*, double*, double)

        define double @run_pid(double %p_gain, double %i_gain, double %d_gain,
                            double %error, double* %integral_state, double* %prev_error, double %dt) {
        entry:
            %res = call double @pid_controller(double %p_gain, double %i_gain, double %d_gain,
                                            double %error, double* %integral_state, double* %prev_error, double %dt)
            ret double %res
        }
    )";

    FuncData p_controller = {
        "p_controller",
        p_c_ir,
        "libcontrollers.so",
        {"double", "p_control_sig"},
        {{"double", "p_gain"},
         {"double", "error"}}
    };
    FuncData pi_controller = {
        "pi_controller",
        pi_c_ir,
        "libcontrollers.so",
        {"double", "pi_control_sig"},
        {{"double", "p_gain"},
         {"double", "i_gain"},
         {"double", "error"},
         {"double*", "integral_state"},
         {"double", "dt"}}
    };
    FuncData pid_controller = {
        "pid_controller",
        pid_c_ir,
        "libcontrollers.so",
        {"double", "pid_control_sig"},
        {{"double", "p_gain"},
         {"double", "i_gain"},
         {"double", "d_gain"},
         {"double", "error"},
         {"double*", "integral_state"},
         {"double*", "prev_error"},
         {"double", "dt"}}
    };
    std::vector<FuncData> controllers = {p_controller, pi_controller, pid_controller};

    // Load external controllers library
    const auto pkg = "jit_control_arch";
    const auto prefix = ament_index_cpp::get_package_prefix(pkg) + "/lib/";
    const auto lib_controllers_path = prefix + "libcontrollers.so";
    if (llvm::sys::DynamicLibrary::LoadLibraryPermanently(lib_controllers_path.c_str())) {
        RCLCPP_ERROR(get_logger(), "Failed to load controllers library: %s", lib_controllers_path.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "Loaded controllers library: %s", lib_controllers_path.c_str());

    // Load variables IR
    load_ir(variables_c_ir, *JIT);

    for (const auto &controller : controllers) {
        // Load IR
        load_ir(controller.ir.c_str(), *JIT);

        // Lookup function
        auto Sym = cantFail(JIT->lookup(controller.name));
        void* fn_ptr = Sym.toPtr<void*>();
        RCLCPP_INFO(get_logger(), "Located function: %s at %p", controller.name.c_str(), fn_ptr);

        // Gather argument pointers from global variables in the JIT
        std::vector<void*> args;
        args.reserve(controller.args.size());

        // Stable storage for pointer-typed argument *values*
        std::vector<void*> pointer_values;
        pointer_values.reserve(controller.args.size());

        for (const auto &arg : controller.args) {
            auto gv = cantFail(JIT->lookup(arg.name));
            void* var_addr = gv.toPtr<void*>(); // address of the JIT global's storage
            if (!arg.type.empty() && arg.type.back() == '*') {
                pointer_values.push_back(var_addr);                // store the pointer value
                void** ptr_to_value = &pointer_values.back();      // stable address to pass to libffi
                args.push_back(static_cast<void*>(ptr_to_value));
            } else {
                args.push_back(var_addr);
            }
        }

        // Call function
        try {
            std::any result = callFunction(fn_ptr, controller.returnSig, controller.args, args);
            if (controller.returnSig.type == "double") {
                double res = std::any_cast<double>(result);
                RCLCPP_INFO(get_logger(), "Controller %s output: %f", controller.name.c_str(), res);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error calling function %s: %s", controller.name.c_str(), e.what());
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LLVMJITNode>();
    node->test();
    rclcpp::shutdown();
    return 0;
}

