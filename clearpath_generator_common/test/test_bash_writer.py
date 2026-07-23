# Software License Agreement (BSD)
#
# @author    Toni Baltovski <tbaltovski@clearpathrobotics.com>
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
import os
import tempfile

from clearpath_generator_common.bash.writer import BashWriter
from clearpath_generator_common.common import BashFile


class TestBashWriter:

    def test_add_export_simple_value(self):
        """Test exporting a simple string value."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            writer.add_export('TEST_VAR', 'simple_value')
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            assert 'export TEST_VAR="simple_value"' in content

    def test_add_export_with_double_quotes(self):
        """Test exporting a value containing double quotes."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            zenoh_val = 'mode="client";connect/endpoints=["tcp/10.27.10.97:7447"]'
            writer.add_export('ZENOH_CONFIG_OVERRIDE', zenoh_val)
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            expected = "export ZENOH_CONFIG_OVERRIDE='" + zenoh_val + "'"
            assert expected in content

    def test_add_export_with_single_quotes(self):
        """Test exporting a value containing single quotes."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            writer.add_export('TEST_VAR', "it's a test")
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            assert 'export TEST_VAR="it\'s a test"' in content

    def test_add_export_with_both_quotes(self):
        """Test exporting a value with both quote types."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            writer.add_export('TEST_VAR', 'it\'s "quoted"')
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            assert 'export TEST_VAR="it\'s \\"quoted\\""' in content

    def test_add_export_already_quoted_double(self):
        """Test exporting a value already wrapped in double quotes."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            writer.add_export('TEST_VAR', '"already_quoted"')
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            assert 'export TEST_VAR="already_quoted"' in content

    def test_add_export_already_quoted_single(self):
        """Test exporting a value already wrapped in single quotes."""
        with tempfile.TemporaryDirectory() as tmpdir:
            bash_file = BashFile(os.path.join(tmpdir, 'test.bash'))
            writer = BashWriter(bash_file)
            writer.add_export('TEST_VAR', "'already_quoted'")
            writer.close()

            with open(bash_file.full_path, 'r') as f:
                content = f.read()

            assert "export TEST_VAR='already_quoted'" in content
