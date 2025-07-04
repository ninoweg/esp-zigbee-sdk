# SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

# pylint: disable=W0621  # redefined-outer-name

# This file is a pytest root configuration file and provide the following functionalities:
# 1. Defines a few fixtures that could be used under the whole project.
# 2. Defines a few hook functions.
#
# IDF is using [pytest](https://github.com/pytest-dev/pytest) and
# [pytest-embedded plugin](https://github.com/espressif/pytest-embedded) as its example test framework.
#
# This is an experimental feature, and if you found any bug or have any question, please report to
# https://github.com/espressif/pytest-embedded/issues

import logging
import os
import xml.etree.ElementTree as ET
from datetime import datetime
from typing import Callable, List, Optional, Tuple

import pytest
from _pytest.config import Config, ExitCode
from _pytest.fixtures import FixtureRequest
from _pytest.main import Session
from _pytest.nodes import Item
from _pytest.python import Function
from _pytest.reports import TestReport
from _pytest.runner import CallInfo
from _pytest.terminal import TerminalReporter
from pytest_embedded.plugin import multi_dut_argument, multi_dut_fixture
from pytest_embedded.utils import find_by_suffix
import time
import subprocess
import copy

DEFAULT_SDKCONFIG = 'default'

ESPPORT1 = os.getenv('ESPPORT1')
ESPPORT2 = os.getenv('ESPPORT2')
ESPPORT3 = os.getenv('ESPPORT3')
ESPPORT4 = os.getenv('ESPPORT4')
ESPPORT5 = os.getenv('ESPPORT5')
ESPPORT6 = os.getenv('ESPPORT6')
ESPPORT7 = os.getenv('ESPPORT7')
ESPPORT8 = os.getenv('ESPPORT8')
ESPPORT9 = os.getenv('ESPPORT9')

PORT_MAPPING = {'esp32s3': [ESPPORT3],
                'esp32h2': [ESPPORT1, ESPPORT2],
                'esp32c6': [ESPPORT4, ESPPORT5],
                'esp32c5': [ESPPORT6, ESPPORT7],}
PORT_MAPPING_GATEWAY = {'esp32s3': [ESPPORT8],
                        'esp32h2': [ESPPORT1, ESPPORT9]}

def pytest_generate_tests(metafunc):
    logging.info(f"Generating test for: {metafunc.function.__name__}")
    port_mapping = copy.deepcopy(PORT_MAPPING)

    target_option = metafunc.config.getoption('target')
    all_target_marks = {m.name for m in metafunc.definition.own_markers}
    if target_option not in all_target_marks:
        return

    targets = []
    ports = []
    # Fix the device port for the dual-chip gateway
    if 'dual_chip_gateway' in all_target_marks:
        ports = [PORT_MAPPING_GATEWAY['esp32h2'][0], PORT_MAPPING_GATEWAY['esp32h2'][1], PORT_MAPPING_GATEWAY['esp32s3'][0]]
        if not all(isinstance(p, str) and p.strip() for p in ports):
            raise ValueError(f'Environment variable for target {targets} port is not set')
        port_str = '|'.join(ports)
        logging.info(f"port: {port_str}")
        metafunc.parametrize('port', [port_str], indirect=True)
        return
    # target string e.g. 'esp32h2|esp32h2'
    for mark in metafunc.definition.iter_markers(name='parametrize'):
        if not mark.args:
            continue
        arg_names = [name.strip() for name in mark.args[0].split(',')]
        param_values = mark.args[1][0]
        if 'target' not in arg_names:
            count_index = arg_names.index('count')
            count_val = param_values[count_index]
            targets = [target_option] * count_val
        else:
            target_index = arg_names.index('target')
            target_string = param_values[target_index]
            targets = target_string.split('|')
    if not targets:
        raise ValueError(f'No targets get from marks')
    for t in targets:
        if t not in port_mapping:
            raise ValueError(f'Target {t} not found in PORT_MAPPING')
        if not port_mapping[t]:
            raise ValueError(f'No available ports left for target {t}')
        port = port_mapping[t].pop(0)
        if not port:
            raise ValueError(f'Environment variable for target {t} port is not set')
        ports.append(port)

    port_str = '|'.join(ports)
    logging.info(f"port: {port_str}")
    metafunc.parametrize('port', [port_str], indirect=True)

##################
# Help Functions #
##################
def format_case_id(target: Optional[str], config: Optional[str], case: str) -> str:
    return f'{target}.{config}.{case}'


def item_marker_names(item: Item) -> List[str]:
    return [marker.name for marker in item.iter_markers()]


############
# Fixtures #
############
@pytest.fixture(scope='session', autouse=True)
def session_tempdir() -> str:
    _tmpdir = os.path.join(
        os.path.dirname(__file__),
        'pytest_embedded_log',
        datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
    )
    os.makedirs(_tmpdir, exist_ok=True)
    return _tmpdir


@pytest.fixture
@multi_dut_argument
def config(request: FixtureRequest) -> str:
    return getattr(request, 'param', None) or DEFAULT_SDKCONFIG


@pytest.fixture
def test_func_name(request: FixtureRequest) -> str:
    return request.node.function.__name__  # type: ignore


@pytest.fixture
def test_case_name(request: FixtureRequest, target: str, config: str) -> str:
    return format_case_id(target, config, request.node.originalname)


@pytest.fixture
@multi_dut_fixture
def build_dir(app_path: str, target: Optional[str], config: Optional[str]) -> str:
    """
    Check local build dir with the following priority:

    1. build_<target>_<config>
    2. build_<target>
    3. build_<config>
    4. build

    Args:
        app_path: app path
        target: target
        config: config

    Returns:
        valid build directory
    """

    check_dirs = []
    if target is not None and config is not None:
        check_dirs.append(f'build_{target}_{config}')
    if target is not None:
        check_dirs.append(f'build_{target}')
    if config is not None:
        check_dirs.append(f'build_{config}')
    check_dirs.append('build')

    for check_dir in check_dirs:
        binary_path = os.path.join(app_path, check_dir)
        if os.path.isdir(binary_path):
            logging.info(f'find valid binary path: {binary_path}')
            return check_dir

        logging.warning(
            'checking binary path: %s... missing... try another place', binary_path
        )

    recommend_place = check_dirs[0]
    raise ValueError(
        f'no build dir valid. Please build the binary via "idf.py -B {recommend_place} build" and run pytest again'
    )


@pytest.fixture(autouse=True)
@multi_dut_fixture
def junit_properties(
        test_case_name: str, record_xml_attribute: Callable[[str, object], None]
) -> None:
    """
    This fixture is autoused and will modify the junit report test case name to <target>.<config>.<case_name>
    """
    record_xml_attribute('name', test_case_name)


@pytest.fixture()
def teardown_fixture(dut):
    """
    A pytest fixture responsible for erasing the flash memory of a list of test devices post-testing.
    Yields:
        None: The fixture initially yields control back to the test function without any modification to 'dut'.
    Returns:
        None: No return value since the primary purpose is the teardown side effect.
    """
    yield
    # after test, close dut monitor, and do erase flash process
    serial_port_list = []
    for device in dut:
        device.serial.close()
        serial_port_list.append(device.serial.port)
    proc = None

    def erase_flash(serial_port):
        proc = subprocess.Popen(f'python -m esptool --port {serial_port} erase_flash', shell=True)
        proc.wait()
        if proc.returncode != 0:
            logging.warning(f"Erase failed on {serial_port}: {proc.returncode}")
            return False
        return True
    # Erase flash on all ports, and retry if failed
    failed_ports = []
    for serial_port in serial_port_list:
        logging.info(f'erase flash on serial_port: {serial_port}')
        if not erase_flash(serial_port):
            failed_ports.append(serial_port)

    if failed_ports:
        for port in failed_ports:
            if not erase_flash(port):
                logging.warning(f"Failed to erase flash on {port} again")
    if proc is not None:
        proc.kill()
    time.sleep(1)

@pytest.fixture(scope='function', autouse=True)
def erase_esp32s3_port(request):
    """
    Ensures ESP32-S3 is idle to avoid flashing issues on ESP32-H2.
    """
    if PORT_MAPPING_GATEWAY['esp32s3'][0]:
        try:
            subprocess.run(['python', '-m', 'esptool', '--port', PORT_MAPPING_GATEWAY['esp32s3'][0], 'erase_flash'], check=True)
        except subprocess.CalledProcessError as e:
            logging.info(f"Failed to erase ESP32S3 port {PORT_MAPPING_GATEWAY['esp32s3'][0]}: {e}")


##################
# Hook functions #
##################
_idf_pytest_embedded_key = pytest.StashKey['IdfPytestEmbedded']


def pytest_configure(config: Config) -> None:
    # cli option "--target"
    target = config.getoption('target') or ''

    help_commands = ['--help', '--fixtures', '--markers', '--version']
    for cmd in help_commands:
        if cmd in config.invocation_params.args:
            target = 'unneeded'
            break

    assert target, "Must specify target by --target"

    config.stash[_idf_pytest_embedded_key] = IdfPytestEmbedded(
        target=target,
    )
    config.pluginmanager.register(config.stash[_idf_pytest_embedded_key])


def pytest_unconfigure(config: Config) -> None:
    _pytest_embedded = config.stash.get(_idf_pytest_embedded_key, None)
    if _pytest_embedded:
        del config.stash[_idf_pytest_embedded_key]
        config.pluginmanager.unregister(_pytest_embedded)


class IdfPytestEmbedded:
    def __init__(
            self,
            target: Optional[str] = None,
    ):
        # CLI options to filter the test cases
        self.target = target
        self._failed_cases: List[
            Tuple[str, bool, bool]
        ] = []  # (test_case_name, is_known_failure_cases, is_xfail)

    @property
    def failed_cases(self) -> List[str]:
        return [
            case
            for case, is_xfail in self._failed_cases
            if not is_xfail
        ]

    @property
    def xfail_cases(self) -> List[str]:
        return [case for case, is_xfail in self._failed_cases if is_xfail]

    @pytest.hookimpl(tryfirst=True)
    def pytest_sessionstart(self, session: Session) -> None:
        if self.target:
            self.target = self.target.lower()
            session.config.option.target = self.target

    @pytest.hookimpl(tryfirst=True)
    def pytest_collection_modifyitems(self, items: List[Function]) -> None:
        # sort by file path and callspec.config
        # implement like this since this is a limitation of pytest, couldn't get fixture values while collecting
        # https://github.com/pytest-dev/pytest/discussions/9689
        def _get_param_config(_item: Function) -> str:
            if hasattr(_item, 'callspec'):
                return _item.callspec.params.get('config', DEFAULT_SDKCONFIG)  # type: ignore
            return DEFAULT_SDKCONFIG

        items.sort(key=lambda x: (os.path.dirname(x.path), _get_param_config(x)))

        # set default timeout 10 minutes for each case
        for item in items:
            if 'timeout' not in item.keywords:
                item.add_marker(pytest.mark.timeout(10 * 60))

        # filter all the test cases with "--target"
        if self.target:
            items[:] = [
                item for item in items if self.target in item_marker_names(item)
            ]

    def pytest_runtest_makereport(
            self, item: Function, call: CallInfo[None]
    ) -> Optional[TestReport]:
        report = TestReport.from_item_and_call(item, call)
        if report.outcome == 'failed':
            test_case_name = item.funcargs.get('test_case_name', '')
            is_xfail = report.keywords.get('xfail', False)
            self._failed_cases.append((test_case_name, is_xfail))

        return report

    @pytest.hookimpl(trylast=True)
    def pytest_runtest_teardown(self, item: Function) -> None:
        """
        Format the test case generated junit reports
        """
        tempdir = item.funcargs.get('test_case_tempdir')
        if not tempdir:
            return

        junits = find_by_suffix('.xml', tempdir)
        if not junits:
            return

        target = item.funcargs['target']
        config = item.funcargs['config']
        for junit in junits:
            xml = ET.parse(junit)
            testcases = xml.findall('.//testcase')
            for case in testcases:
                case.attrib['name'] = format_case_id(
                    target, config, case.attrib['name']
                )
                if 'file' in case.attrib:
                    case.attrib['file'] = case.attrib['file'].replace(
                        '/IDF/', ''
                    )  # our unity test framework
            xml.write(junit)

    def pytest_sessionfinish(self, session: Session, exitstatus: int) -> None:
        if exitstatus != 0:
            if exitstatus == ExitCode.NO_TESTS_COLLECTED:
                session.exitstatus = 0

    def pytest_terminal_summary(self, terminalreporter: TerminalReporter) -> None:
        if self.xfail_cases:
            terminalreporter.section('xfail cases', bold=True, yellow=True)
            terminalreporter.line('\n'.join(self.xfail_cases))

        if self.failed_cases:
            terminalreporter.section('Failed cases', bold=True, red=True)
            terminalreporter.line('\n'.join(self.failed_cases))
