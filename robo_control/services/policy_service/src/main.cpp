#include "policy_service/policy_service.hpp"

#include <transport/service_main.hpp>

int main(int argc, char **argv) {
  transport::ServiceOptions options;
  options.lock_memory = true;
  return transport::run_service<policy_service::PolicyService>(
      "policy_service", argc, argv, options);
}
