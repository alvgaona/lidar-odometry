#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <any>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace lidarodom {

class Params;

namespace detail {

// Forward declaration for vector to_string
template <typename T>
std::string to_string(const T& val);

template <typename T>
std::string vector_to_string(const std::vector<T>& vec) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << to_string(vec[i]);
    }
    oss << "]";
    return oss.str();
}

template <typename T>
std::string to_string(const T& val) {
    if constexpr (std::is_same_v<T, bool>) {
        return val ? "true" : "false";
    } else if constexpr (std::is_same_v<T, std::string>) {
        return "\"" + val + "\"";
    } else if constexpr (std::is_same_v<T, double>) {
        std::ostringstream oss;
        oss << std::defaultfloat << val;
        return oss.str();
    } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
        return vector_to_string(val);
    } else if constexpr (std::is_same_v<T, std::vector<int64_t>>) {
        return vector_to_string(val);
    } else if constexpr (std::is_same_v<T, std::vector<double>>) {
        return vector_to_string(val);
    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
        return vector_to_string(val);
    } else {
        return std::to_string(val);
    }
}

template <typename T>
std::string type_name() {
    if constexpr (std::is_same_v<T, bool>) return "bool";
    else if constexpr (std::is_same_v<T, int>) return "int";
    else if constexpr (std::is_same_v<T, int64_t>) return "int64";
    else if constexpr (std::is_same_v<T, double>) return "double";
    else if constexpr (std::is_same_v<T, std::string>) return "string";
    else if constexpr (std::is_same_v<T, std::vector<bool>>) return "bool[]";
    else if constexpr (std::is_same_v<T, std::vector<int64_t>>) return "int64[]";
    else if constexpr (std::is_same_v<T, std::vector<double>>) return "double[]";
    else if constexpr (std::is_same_v<T, std::vector<std::string>>) return "string[]";
    else return "unknown";
}

struct ParamEntry {
    std::string name;
    std::string help;
    std::string type_name;
    std::string default_str;
    std::string constraints;
    std::function<void(rclcpp::Node*, std::unordered_map<std::string, std::any>&)> declare_fn;
    std::function<void()> validate_fn;
    std::function<std::string()> value_str_fn;
};

}  // namespace detail

template <typename T>
class ParamBuilder {
public:
    explicit ParamBuilder(const std::string& name) : name_(name) {}

    ParamBuilder& value(const T& default_val) {
        default_value_ = default_val;
        return *this;
    }

    ParamBuilder& help(const std::string& description) {
        help_ = description;
        return *this;
    }

    template <typename U = T>
    std::enable_if_t<std::is_arithmetic_v<U> && !std::is_same_v<U, bool>, ParamBuilder&>
    range(U min_val, U max_val) {
        range_min_ = static_cast<double>(min_val);
        range_max_ = static_cast<double>(max_val);
        return *this;
    }

    template <typename U = T>
    std::enable_if_t<std::is_same_v<U, std::string>, ParamBuilder&>
    choices(std::initializer_list<std::string> valid_choices) {
        choices_ = std::vector<std::string>(valid_choices);
        return *this;
    }

private:
    friend class Params;

    std::string name_;
    std::optional<T> default_value_;
    std::string help_;
    std::optional<double> range_min_;
    std::optional<double> range_max_;
    std::optional<std::vector<std::string>> choices_;

    void finalize(Params& params);
};

class Params {
public:
    // Construct without node (for help-only mode)
    Params() : node_(nullptr) {}

    // Construct with node (standard usage)
    explicit Params(rclcpp::Node* node) : node_(node) {}

    template <typename T>
    ParamBuilder<T>& bind(const std::string& name) {
        auto builder = std::make_shared<ParamBuilder<T>>(name);
        pending_builders_.push_back([b = builder](Params& p) { b->finalize(p); });
        builder_storage_.push_back(builder);
        return *builder;
    }

    // Parse with node provided at construction
    void parse() {
        if (!node_) {
            throw std::runtime_error("Params::parse() requires a node. Use parse(node) or construct with Params(node).");
        }
        parse(node_);
    }

    // Parse with explicit node (for deferred node binding)
    void parse(rclcpp::Node* node) {
        node_ = node;

        for (auto& finalize_fn : pending_builders_) {
            finalize_fn(*this);
        }
        pending_builders_.clear();

        for (auto& entry : entries_) {
            entry.declare_fn(node_, values_);
        }

        for (auto& entry : entries_) {
            entry.validate_fn();
        }

        log_summary();
    }

    template <typename T>
    T get(const std::string& name) const {
        auto it = values_.find(name);
        if (it == values_.end()) {
            throw std::runtime_error("Parameter '" + name + "' not found");
        }
        try {
            return std::any_cast<T>(it->second);
        } catch (const std::bad_any_cast&) {
            throw std::runtime_error(
                "Parameter '" + name + "' type mismatch: requested " +
                detail::type_name<T>());
        }
    }

    void print_help(const std::string& description = "",
                    const std::string& package = "<package>",
                    const std::string& executable = "<executable>") const {
        std::cout << format_help(description, package, executable);
    }

    std::string help_string(const std::string& description = "",
                            const std::string& package = "<package>",
                            const std::string& executable = "<executable>") const {
        return format_help(description, package, executable);
    }

private:
    std::string format_help(const std::string& description,
                            const std::string& package,
                            const std::string& executable) const {
        finalize_if_needed();

        std::ostringstream oss;

        // Description
        if (!description.empty()) {
            oss << description << "\n\n";
        }

        // Usage
        oss << "Usage: ros2 run " << package << " " << executable << " --ros-args [PARAMETERS]\n";
        oss << "       ros2 run " << package << " " << executable << " --ros-args --params-file <FILE>\n";

        // Calculate column width for alignment
        size_t max_param_width = 0;
        for (const auto& e : entries_) {
            // Format: "-p name:=<type>"
            size_t width = 3 + e.name.length() + 3 + e.type_name.length() + 1;  // "-p " + name + ":=<" + type + ">"
            max_param_width = std::max(max_param_width, width);
        }
        max_param_width += 4;  // padding

        // Parameters
        oss << "\nParameters:\n";

        for (const auto& e : entries_) {
            std::ostringstream param_col;
            param_col << "-p " << e.name << ":=<" << e.type_name << ">";

            oss << "  " << std::left << std::setw(max_param_width) << param_col.str();
            oss << e.help;

            // Add metadata inline
            std::string meta;
            if (!e.default_str.empty()) {
                meta += "[default: " + e.default_str + "]";
            }
            if (!e.constraints.empty()) {
                if (!meta.empty()) meta += " ";
                meta += "[" + e.constraints + "]";
            }
            if (!meta.empty()) {
                if (!e.help.empty()) oss << " ";
                oss << meta;
            }
            oss << "\n";
        }

        return oss.str();
    }

public:

    // Check if --help was passed in command line args
    static bool has_help_flag(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if (arg == "--help" || arg == "-h") {
                return true;
            }
        }
        return false;
    }

private:
    template <typename T>
    friend class ParamBuilder;

    rclcpp::Node* node_;
    mutable std::vector<detail::ParamEntry> entries_;
    std::unordered_map<std::string, std::any> values_;
    mutable std::vector<std::function<void(Params&)>> pending_builders_;
    std::vector<std::shared_ptr<void>> builder_storage_;

    void finalize_if_needed() const {
        for (auto& finalize_fn : pending_builders_) {
            finalize_fn(const_cast<Params&>(*this));
        }
        pending_builders_.clear();
    }

    template <typename T>
    void register_param(const std::string& name,
                        const std::optional<T>& default_value,
                        const std::string& help,
                        std::optional<double> range_min,
                        std::optional<double> range_max,
                        std::optional<std::vector<std::string>> choices) {
        detail::ParamEntry entry;
        entry.name = name;
        entry.help = help;
        entry.type_name = detail::type_name<T>();

        // Store default value string for help
        if (default_value.has_value()) {
            entry.default_str = detail::to_string(*default_value);
        } else {
            entry.default_str = "(none)";
        }

        // Store constraints string for help
        std::ostringstream constraints;
        if (range_min.has_value() && range_max.has_value()) {
            constraints << "range: " << *range_min << ".." << *range_max;
        }
        if (choices.has_value()) {
            if (!constraints.str().empty()) constraints << ", ";
            constraints << "options: ";
            for (size_t i = 0; i < choices->size(); ++i) {
                if (i > 0) constraints << "|";
                constraints << (*choices)[i];
            }
        }
        entry.constraints = constraints.str();

        auto value_ptr = std::make_shared<T>();

        entry.declare_fn = [=](rclcpp::Node* node,
                               std::unordered_map<std::string, std::any>& values) {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.description = help;

            if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
                if (range_min.has_value() && range_max.has_value()) {
                    if constexpr (std::is_integral_v<T>) {
                        desc.integer_range.resize(1);
                        desc.integer_range[0].from_value = static_cast<int64_t>(*range_min);
                        desc.integer_range[0].to_value = static_cast<int64_t>(*range_max);
                    } else {
                        desc.floating_point_range.resize(1);
                        desc.floating_point_range[0].from_value = *range_min;
                        desc.floating_point_range[0].to_value = *range_max;
                    }
                }
            }

            T default_val{};
            if (default_value.has_value()) {
                default_val = *default_value;
            }
            *value_ptr = node->declare_parameter<T>(name, default_val, desc);
            values[name] = *value_ptr;
        };

        entry.validate_fn = [=]() {
            if constexpr (std::is_same_v<T, std::string>) {
                if (choices.has_value()) {
                    const auto& valid = *choices;
                    if (std::find(valid.begin(), valid.end(), *value_ptr) == valid.end()) {
                        std::ostringstream oss;
                        oss << "Parameter '" << name << "' value \"" << *value_ptr
                            << "\" not in choices: [";
                        for (size_t i = 0; i < valid.size(); ++i) {
                            if (i > 0) oss << ", ";
                            oss << "\"" << valid[i] << "\"";
                        }
                        oss << "]";
                        throw std::invalid_argument(oss.str());
                    }
                }
            }
        };

        entry.value_str_fn = [value_ptr]() {
            return detail::to_string(*value_ptr);
        };

        entries_.push_back(std::move(entry));
    }

    void log_summary() const {
        if (entries_.empty()) return;

        size_t name_w = 4, type_w = 4, val_w = 5;
        for (const auto& e : entries_) {
            name_w = std::max(name_w, e.name.length());
            type_w = std::max(type_w, e.type_name.length());
            val_w = std::max(val_w, e.value_str_fn().length());
        }
        name_w += 2;
        type_w += 2;
        val_w += 2;

        RCLCPP_INFO(node_->get_logger(), "Parameters:");
        for (const auto& e : entries_) {
            std::ostringstream oss;
            oss << "  " << std::left
                << std::setw(name_w) << e.name
                << std::setw(type_w) << e.type_name
                << std::setw(val_w) << e.value_str_fn();
            if (!e.help.empty()) {
                oss << " # " << e.help;
            }
            RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
        }
    }
};

template <typename T>
void ParamBuilder<T>::finalize(Params& params) {
    params.register_param<T>(name_, default_value_, help_,
                              range_min_, range_max_, choices_);
}

}  // namespace lidarodom
