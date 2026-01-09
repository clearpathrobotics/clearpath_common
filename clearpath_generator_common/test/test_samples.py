# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2026, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of Clearpath Robotics.

"""Pytest Test Class and Utility Methods for Validating Generated Samples."""

import difflib
import filecmp
import os

from typing import List

from ament_index_python.packages import get_package_share_directory

from .test_generate_samples import generate_test_samples, get_test_samples


class MissingSampleException(Exception):
    """Exception to capture missing sample directories."""

    def __init__(self, message, errors):
        """Initialize default exception and keep errors."""
        super().__init__(message)
        self.errors = errors


class MismatchSampleException(Exception):
    """Exception to capture mismatching sample files."""

    def __init__(self, message, errors):
        """Initialize default exception and keep errors."""
        super().__init__(message)
        self.errors = errors


class TestSamples:
    """
    Test class for validating generated samples.

    Methods
    -------
    test_generate_samples()
        Generate samples. Throw exception if failed.
    test_number_of_samples()
        Validate the same number of samples are generated as installed.
    test_samples_match()
        Validate generated samples match existing files.

    """

    new_sample_dir = os.path.join(os.environ['HOME'], '.clearpath', 'samples')
    share_dir = get_package_share_directory('clearpath_generator_common')
    installed_sample_dir = os.path.join(share_dir, 'samples')
    # Find real non-symbolic path in case of --symlink-install
    for sample in os.listdir(installed_sample_dir):
        real_installed_sample_dir = os.path.dirname(
            os.path.dirname(
                os.path.realpath(
                    os.path.join(installed_sample_dir, sample, 'robot.yaml'))))
        break
    installed_sample_dir = real_installed_sample_dir
    relpath_installed_sample_dir = os.sep.join(
        os.path.normpath(
            installed_sample_dir).split(os.sep)[-2:])
    relpath_new_sample_dir = os.sep.join(
        os.path.normpath(
            new_sample_dir).split(os.sep)[-2:])
    found_samples = len(get_test_samples()) > 0

    def filter_lines(self, lines: List[str], filepath: str) -> str:
        """Filter line files to prevent comparing lines that are expected to be different."""
        filtered = []

        if 'setup.bash' == os.path.basename(filepath):
            if 'Bash setup generated' in lines[0]:
                filtered = lines[1:]

        elif ('test_all_dual_manipulators' in filepath
                and 'robot.srdf' == os.path.basename(filepath)):
            for line in lines:
                if 'disable_collisions' in line:
                    break
                filtered.append(line)
            filtered.extend(lines[-2:])
        return filtered

    def diff_dir_trees(self, dir_1: str, dir_2: str, shallow: bool = False) -> List:
        """Compare the two directory trees and return a list of differences."""
        logs = []
        # Compare Directories
        dirs_cmp = filecmp.dircmp(dir_1, dir_2)
        # Log Only in Installed Directory
        if len(dirs_cmp.left_only) > 0:
            logs.append(
                f'Files/directories: {dirs_cmp.left_only}, '
                f'only found in: {dir_1} '
                f'not in: {dir_2}'
            )
        # Log Only in Generated Directory
        if len(dirs_cmp.right_only) > 0:
            logs.append(
                f'Files/directories: {dirs_cmp.right_only}, '
                f'only found in: {dir_2} '
                f'not in: {dir_1}'
            )
        # Compare Files
        (_, mismatches, errors) = filecmp.cmpfiles(
            dir_1, dir_2, dirs_cmp.common_files, shallow=False)
        # Log File Mismatches
        for mismatch in mismatches:
            path_1 = os.path.join(dir_1, mismatch)
            path_2 = os.path.join(dir_2, mismatch)
            with open(path_1, 'r') as fp1:
                lines_1 = fp1.readlines()
            with open(path_2, 'r') as fp2:
                lines_2 = fp2.readlines()
            lines_1 = self.filter_lines(lines_1, path_1)
            lines_2 = self.filter_lines(lines_2, path_2)
            file_diff = difflib.unified_diff(
                a=lines_1,
                b=lines_2,
                fromfile=path_1,
                tofile=path_2,
            )
            str_file_diff = ''    ''.join(file_diff)
            if len(str_file_diff) > 0:
                logs.append(f'File mismatch: \n{str_file_diff}')
        # Log File Errors
        if len(errors) > 0:
            logs.append(
                f'Errors: {errors} found when '
                f'comparing: {dir_1} '
                f'and: {dir_2}'
            )
        # Recurse
        if not shallow:
            for common_dir in dirs_cmp.common_dirs:
                sub_log = self.diff_dir_trees(
                    os.path.join(dir_1, common_dir),
                    os.path.join(dir_2, common_dir)
                )
                logs.extend(sub_log)
        return logs

    def test_generate_samples(self):
        """Validate sample generation."""
        generate_test_samples(self.new_sample_dir)

    def test_number_of_samples_match(self):
        """Validate number of samples matches."""
        if not self.found_samples:
            return
        errors = self.diff_dir_trees(
            self.new_sample_dir,
            self.installed_sample_dir,
            shallow=True)

        if len(errors) > 0:
            raise MissingSampleException(
                'The number of generated samples does not match installed samples:\n'
                f'\n{"\n".join(errors)}',
                errors
            )

    def test_samples_match(self):
        """Validate contents of generated sample directory match."""
        if not self.found_samples:
            return
        dirs_cmp = filecmp.dircmp(
            self.new_sample_dir,
            self.installed_sample_dir
        )
        errors = []
        for common_dir in dirs_cmp.common_dirs:
            sample_errors = self.diff_dir_trees(
                os.path.join(self.new_sample_dir, common_dir),
                os.path.join(self.installed_sample_dir, common_dir)
            )
            if len(sample_errors) > 0:
                errors.append(
                    f'Sample "{common_dir}" mismatch between installed and generated files:\n\n'
                    f'{"\n\n".join(sample_errors)}'
                )

        if len(errors) > 0:
            raise MismatchSampleException(
                f'{len(errors)} generated sample(s) did not match installed samples'
                f'\n{"\n\n".join(errors)}',
                errors
            )
