import pytest
from rclpy.node import Node
import rclpy

# Import the ExampleNode class
from orca_extend.example_node import ExampleNode


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def example_node(rclpy_init):
    node = ExampleNode()
    yield node
    node.destroy_node()


def test_initialization(example_node):
    assert isinstance(example_node, Node)
    assert hasattr(example_node, 'publisher_')
    assert hasattr(example_node, 'timer')
    assert example_node.i == 0


def test_timer_callback(example_node, mocker):
    mock_publish = mocker.patch.object(example_node.publisher_, 'publish')
    example_node.i = 5

    example_node.timer_callback()

    assert example_node.i == 6
    mock_publish.assert_called_once()


def test_main(mocker):
    mock_init = mocker.patch('rclpy.init')
    mock_spin = mocker.patch('rclpy.spin')
    mock_shutdown = mocker.patch('rclpy.shutdown')

    from orca_extend.example_node import main
    main()

    mock_init.assert_called_once()
    mock_spin.assert_called_once()
    mock_shutdown.assert_called_once()
