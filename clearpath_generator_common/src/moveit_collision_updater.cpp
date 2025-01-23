/**
Software License Agreement (BSD)
\file      moveit_collision_updater.cpp
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <moveit_setup_srdf_plugins/default_collisions.hpp>
#include <moveit_setup_framework/data/package_settings_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit/rdf_loader/rdf_loader.hpp>
#include <boost/program_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  std::filesystem::path urdf_path;
  std::filesystem::path srdf_path;
  std::filesystem::path output_path;
  std::vector<std::string> xacro_args;

  bool include_default = false, include_always = false, keep_old = false, verbose = false;

  double min_collision_fraction = 1.0;

  uint32_t never_trials = 0;

  // clang-format off
  po::options_description desc("Allowed options");
  desc.add_options()("help", "show help")(
    "urdf", po::value(&urdf_path),
    "path to URDF ( or xacro)")(
    "srdf", po::value(&srdf_path),
    "path to SRDF ( or xacro)")(
    "output", po::value(&output_path),
    "output path for SRDF")(
    "xacro-args", po::value<std::vector<std::string>>()->composing(),
    "additional arguments for xacro")(
    "default", po::bool_switch(&include_default),
    "disable default colliding pairs")(
    "always", po::bool_switch(&include_always),
    "disable always colliding pairs")(
    "keep", po::bool_switch(&keep_old),
    "keep disabled link from SRDF")(
    "verbose", po::bool_switch(&verbose),
    "verbose output")(
    "trials", po::value(&never_trials),
    "number of trials for searching never colliding pairs")(
    "min-collision-fraction", po::value(&min_collision_fraction),
    "fraction of small sample size to determine links that are always colliding");


  po::positional_options_description pos_desc;
  pos_desc.add("xacro-args", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << '\n';
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("moveit_collision_updater");
  moveit_setup::DataWarehousePtr config_data = std::make_shared<moveit_setup::DataWarehouse>(node);

  moveit_setup::srdf_setup::DefaultCollisions setup_step;
  setup_step.initialize(node, config_data);

  auto config = config_data->get<moveit_setup::URDFConfig>("urdf");
  auto srdf_config = config_data->get<moveit_setup::SRDFConfig>("srdf");

  config->loadFromPath(urdf_path, xacro_args);
  srdf_config->loadSRDFFile(srdf_path);
  setup_step.startGenerationThread(never_trials, min_collision_fraction, verbose);
  int thread_progress;
  int last_progress = 0;
  while ((thread_progress = setup_step.getThreadProgress()) < 100) {
    if (thread_progress - last_progress > 10) {
      last_progress = thread_progress;
    }
  }
  setup_step.joinGenerationThread();

  size_t skip_mask = 0;
  if (!include_default) {
    skip_mask |= (1 << moveit_setup::srdf_setup::DEFAULT);
  }
  if (!include_always) {
    skip_mask |= (1 << moveit_setup::srdf_setup::ALWAYS);
  }
  setup_step.linkPairsToSRDFSorted(skip_mask);

  if (output_path.empty()) {
    srdf_config->write(srdf_config->getPath());
  } else {
    srdf_config->write(output_path);
  }
  return 0;
}
