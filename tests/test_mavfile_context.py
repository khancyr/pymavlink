#!/usr/bin/env python3

"""
test the ambient-connection plumbing that mavextra expression helpers rely on.

Expressions are eval()'d strings, so helpers in mavextra cannot be handed the
connection as an argument and instead call mavutil.current_mavfile().
"""

import os
import unittest

from pymavlink import mavexpression
from pymavlink import mavutil


class CurrentMavfileTest(unittest.TestCase):
    '''mavutil.current_mavfile() resolution order and failure mode'''

    def setUp(self):
        # current_mavfile() falls back to this; make each test independent of
        # connections other tests happen to have opened.
        self._saved_global = mavutil.mavfile_global
        mavutil.mavfile_global = None

    def tearDown(self):
        mavutil.mavfile_global = self._saved_global

    def test_raises_clearly_when_no_connection(self):
        with self.assertRaises(RuntimeError) as caught:
            mavutil.current_mavfile()
        # must name the problem, not surface as AttributeError on None
        self.assertIn('no mavlink connection', str(caught.exception))

    def test_falls_back_to_last_opened_connection(self):
        with mavutil.mavlink_connection('udpin:127.0.0.1:0') as conn:
            self.assertIs(mavutil.current_mavfile(), conn)

    def test_scoped_connection_wins_over_last_opened(self):
        '''a scoped evaluation must not be repointed by a later connection'''
        with mavutil.mavlink_connection('udpin:127.0.0.1:0') as first:
            with mavutil.mavlink_connection('udpin:127.0.0.1:0') as second:
                # opening `second` moved the process-wide fallback
                self.assertIs(mavutil.mavfile_global, second)

                seen = []
                mavexpression.__dict__['_probe'] = lambda: seen.append(mavutil.current_mavfile()) or 1
                try:
                    mavutil.evaluate_expression('_probe()', {}, mav=first)
                    self.assertIs(seen[0], first)

                    # and the scope is popped again afterwards
                    self.assertIs(mavutil.current_mavfile(), second)

                    # an unscoped evaluation still uses the fallback, which is
                    # the path tools/mavgraph.py and tools/mavkml.py take
                    seen.clear()
                    mavutil.evaluate_expression('_probe()', {})
                    self.assertIs(seen[0], second)
                finally:
                    del mavexpression.__dict__['_probe']

    def test_scope_is_popped_even_if_evaluation_raises(self):
        with mavutil.mavlink_connection('udpin:127.0.0.1:0') as conn:
            mavexpression.__dict__['_boom'] = lambda: (_ for _ in ()).throw(ValueError('boom'))
            try:
                # evaluate_expression swallows only NameError/ZeroDivisionError/
                # IndexError, so this propagates through the finally
                with self.assertRaises(ValueError):
                    mavutil.evaluate_expression('_boom()', {}, mav=conn)
            finally:
                del mavexpression.__dict__['_boom']
            # if the token had leaked, this would still be `conn` via the
            # contextvar rather than via the fallback
            mavutil.mavfile_global = None
            with self.assertRaises(RuntimeError):
                mavutil.current_mavfile()


class MavextraAmbientConnectionTest(unittest.TestCase):
    '''mavextra helpers resolve the connection through current_mavfile()'''

    def dataflash_path(self):
        return os.path.join(os.path.dirname(__file__), 'test.BIN')

    def test_condition_evaluation_over_a_log(self):
        '''end-to-end: recv_match(condition=...) passes itself as the scope'''
        with mavutil.mavlink_connection(self.dataflash_path()) as mlog:
            m = mlog.recv_match(type='ATT', condition='ATT.Roll<180', blocking=False)
            self.assertIsNotNone(m)
            self.assertEqual(m.get_type(), 'ATT')

    def test_mavextra_helper_reads_the_scoped_connection(self):
        '''eval'd expression -> mavextra helper -> current_mavfile()

        mavextra.delta() takes its reference time from the ambient connection's
        .timestamp, so the value it returns shows which connection the helper
        resolved.
        '''
        class Stub:
            timestamp = 100.0

        scope = Stub()
        # prime delta()'s cache at t=100 with value 1.0, then step the clock and
        # the value: slope over 1s must be (3.0 - 1.0) / 1.0 == 2.0
        self.assertEqual(
            mavutil.evaluate_expression("delta(1.0, 'ctxtest')", {}, mav=scope), 0)
        scope.timestamp = 101.0
        self.assertEqual(
            mavutil.evaluate_expression("delta(3.0, 'ctxtest')", {}, mav=scope), 2.0)


if __name__ == '__main__':
    unittest.main()
