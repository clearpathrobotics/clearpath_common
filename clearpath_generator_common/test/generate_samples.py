"""Scripts to generate samples using the clearpath_generator_common."""
import argparse
import os
import shutil
import signal
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from clearpath_generator_common.bash.generator import BashGenerator
from clearpath_generator_common.description.generator import DescriptionGenerator
from clearpath_generator_common.discovery_server.generator import DiscoveryServerGenerator
from clearpath_generator_common.semantic_description.generator import SemanticDescriptionGenerator
from clearpath_generator_common.vcan.generator import VirtualCANGenerator
from clearpath_generator_common.zenoh_router.generator import ZenohRouterGenerator
from ros2run.api import get_executable_path


PACKAGE = '<package><name>clearpath_generator_common</name></package>\n'


class GenerationFailureException(Exception):
    """Exception to capture generation failures."""

    def __init__(self, message, errors):
        """Initialize default exception and keep errors."""
        super().__init__(message)
        self.errors = errors


def run_ros2_executable(*, path, argv, prefix=None):
    """Execute a ROS2 executable."""
    cmd = [path] + argv

    # on Windows Python scripts are invokable through the interpreter
    if os.name == 'nt' and path.endswith('.py'):
        cmd.insert(0, sys.executable)

    if prefix is not None:
        cmd = prefix + cmd

    process = subprocess.Popen(cmd, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

    # add signal handler for the parent process, so that we can finalize the child process
    def signal_handler(sig, frame):
        print('[ros2run]', 'Received signal: ', signal.strsignal(sig))
        if process.poll() is None:
            # If child process is running, forward the signal to it
            process.send_signal(sig)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    stdout = ''
    stderr = ''

    while process.returncode is None:
        try:
            out, err = process.communicate()
            stdout += out.decode('utf-8')
            stderr += err.decode('utf-8')
        except KeyboardInterrupt:
            # the subprocess will also receive the signal and should shut down
            # therefore we continue here until the process has finished
            pass
    if process.returncode != 0:
        if -process.returncode in signal.valid_signals() and os.name == 'posix':
            # a negative value -N indicates that the child was terminated by signal N.
            print('[ros2run]', signal.strsignal(-process.returncode))
        else:
            # print general failure message instead.
            print('[ros2run]', 'Process exited with failure %d' % (process.returncode))
    return (process.returncode, stdout, stderr)


def generate_bash(setup_path) -> None:
    """Generate bash environment file."""
    bg = BashGenerator(setup_path)
    bg.generate()


def generate_discovery_server(setup_path: str) -> None:
    """Generate FastDDS discovery server start script."""
    dsg = DiscoveryServerGenerator(setup_path)
    dsg.generate()


def generate_zenoh(setup_path: str) -> bool:
    """Generate Zenoh router start script."""
    zrg = ZenohRouterGenerator(setup_path)
    zrg.generate()


def generate_vcan(setup_path: str) -> bool:
    """Generate VCAN bridge script."""
    vcg = VirtualCANGenerator(setup_path)
    vcg.generate()


def generate_description(setup_path: str) -> bool:
    """Generate robot URDF xacro."""
    dg = DescriptionGenerator(setup_path)
    dg.generate()


def generate_semantic_description(setup_path: str) -> bool:
    """Generate robot SRDF."""
    sdg = SemanticDescriptionGenerator(setup_path)
    sdg.generate()
    # Create pseudo package
    with open(os.path.join(setup_path, 'package.xml'), 'w+') as f:
        f.write(PACKAGE)
    # Update collision matrix
    path = get_executable_path(
        executable_name='moveit_collision_updater',
        package_name='clearpath_generator_common'
    )
    argv = [
        '--urdf', os.path.join(setup_path, 'robot.urdf.xacro'),
        '--srdf', os.path.join(setup_path, 'robot.srdf.xacro'),
        '--output', os.path.join(setup_path, 'robot.srdf'),
        '--trials', '10000',
        '--min-collision-fraction', '0.95',
        '--ros-args', '--log-level', 'fatal'
    ]
    ret, out, err = run_ros2_executable(path=path, argv=argv)
    # Delete pseudo package
    os.remove(os.path.join(setup_path, 'package.xml'))

    if ret == 0:
        print(f'Generated {os.path.join(setup_path)}/robot.srdf')
    else:
        print(f'Failed to generate {os.path.join(setup_path)}/robot.srdf')
        raise Exception(f'Failed semantic generation with \n\tstdout:{out} \n\tstderr: {err}')


def error_log(name: str, sample: str, error: Exception) -> str:
    """Return error entry."""
    return f'{name} failed for sample "{sample}" with error: \n{error}'


def get_test_samples():
    """Return all test samples from Clearpath Config package."""
    samples = []
    share_dir = get_package_share_directory('clearpath_config')
    sample_dir = os.path.join(share_dir, 'sample')
    for sample in os.listdir(sample_dir):
        # Filter for Test Samples
        if 'test' not in sample:
            continue
        samples.append(sample)
    return samples


def generate_test_samples(root_dir: str):
    """Generate all files from common generator."""
    # Iterate through all samples in clearpath_config
    share_dir = get_package_share_directory('clearpath_config')
    sample_dir = os.path.join(share_dir, 'sample')
    sample_errors = []
    for sample in get_test_samples():
        print(f'Generating {sample}'.center(100, '-'))
        # Create Clearpath Directory
        src = os.path.join(sample_dir, sample)
        dst = os.path.join(
            os.path.join(root_dir,  os.path.splitext(os.path.basename(sample))[0]),
            'robot.yaml')
        setup_path = os.path.dirname(dst)

        shutil.rmtree(setup_path, ignore_errors=True)
        os.makedirs(setup_path, exist_ok=True)
        shutil.copy(src, dst)
        errors = []
        # Bash
        try:
            generate_bash(setup_path)
        except Exception as e:
            errors.append(error_log('BashGenerator', sample, e))
        # Discovery Server
        try:
            generate_discovery_server(setup_path)
        except Exception as e:
            errors.append(error_log('DiscoveryServerGenerator', sample, e))
        # Zenoh Router
        try:
            generate_zenoh(setup_path)
        except Exception as e:
            errors.append(error_log('ZenohRouterGenerator', sample, e))
        # VCAN Bridge
        try:
            generate_vcan(setup_path)
        except Exception as e:
            errors.append(error_log('VirtualCANGenerator', sample, e))
        # Description
        try:
            generate_description(setup_path)
        except Exception as e:
            errors.append(error_log('DescriptionGenerator', sample, e))
        # Semantic Description
        try:
            generate_semantic_description(setup_path)
        except Exception as e:
            errors.append(error_log('SemanticDescriptionGenerator', sample, e))
        if len(errors) > 0:
            sample_errors.append(f'Sample "{sample}" failed to generate:\n{'\n  '.join(errors)}')
        print()
    if len(sample_errors) > 0:
        print(f'Generators reported {len(sample_errors)} errors'.center(100, '*'))
        raise GenerationFailureException(
            message=f'Generation failed for {len(sample_errors)} samples:\n'
                    f'{"\n".join(sample_errors)}',
            errors=sample_errors
        )


def main():
    """Generate all files in directory path from argument."""
    default_root_dir = os.path.join(
        os.path.dirname(
            os.path.dirname(
                os.path.realpath(__file__))),
        'samples')
    # Get Root Directory from Args
    parser = argparse.ArgumentParser(
        prog='Clearpath Common Sample Generator',
        description='Generate all common files from test samples in the clearpath_config package.',
    )
    parser.add_argument(
        '--out',
        help='Output directory of generated files.',
        default=default_root_dir,
        required=False)
    args = parser.parse_args()

    root_dir = os.path.abspath(args.out)
    assert os.path.isdir(root_dir), f'Output directory "{root_dir}" does not exist.'

    generate_test_samples(root_dir)


if __name__ == '__main__':
    main()
