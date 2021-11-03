import traceback
import unittest


class FailureTestCase(unittest.TestCase):
    def setUp(self):
        super(FailureTestCase, self).setUp()
        print(self.__class__.__name__, self._testMethodName)
        self._failures = []

    def tearDown(self):
        if len(self._failures) > 0:
            self.fail(str(self._failures))
        super(FailureTestCase, self).tearDown()

    def add_failure(self, msg):
        self._failures.append(msg + '\n' + '\n'.join(traceback.format_stack()))

    def add_failure_assert_equal(self, a, b):
        if a != b:
            self.add_failure('{} != {}'.format(a, b))

    def add_failure_assert_true(self, a):
        if not a:
            self.add_failure('{} != True'.format(a))

    def add_failure_assert_false(self, a):
        if a:
            self.add_failure('{} != False'.format(a))

    def add_failure_assert_in(self, a, b):
        if a not in b:
            self.add_failure('{} not in {}'.format(a, b))
