import logging
import unittest

from basekit_driver.logger.stdout_logger import StdOutLogger


class TestStdOutLogger(unittest.TestCase):
    """Test mockup for stdout logger."""

    def setUp(self):
        self.logger = StdOutLogger()
        # Ensure all log levels are captured
        self.logger.set_level(logging.DEBUG)

    def test_debug(self):
        """Test debug printout."""
        with self.assertLogs(self.logger._logger, level='DEBUG') as log:
            self.logger.debug('Test debug message')
            self.assertIn('Test debug message', log.output[0])

    def test_info(self):
        """Test info output."""
        with self.assertLogs(self.logger._logger, level='INFO') as log:
            self.logger.info('Test info message')
            self.assertIn('Test info message', log.output[0])

    def test_warn(self):
        """Test warn logger."""
        with self.assertLogs(self.logger._logger, level='WARNING') as log:
            self.logger.warn('Test warn message')
            self.assertIn('Test warn message', log.output[0])

    def test_error(self):
        """Test error logger."""
        with self.assertLogs(self.logger._logger, level='ERROR') as log:
            self.logger.error('Test error message')
            self.assertIn('Test error message', log.output[0])


if __name__ == '__main__':
    unittest.main()
