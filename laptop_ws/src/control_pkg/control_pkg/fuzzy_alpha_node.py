"""
Fuzzy Alpha Node.

Subscribes to /vibration_feature, evaluates a Mamdani fuzzy inference
system with three input sets, three output sets, and three rules, and
publishes the defuzzified scaling factor alpha on /alpha as a
std_msgs/Float32 message.

When the fuzzy_enabled parameter is set to False at launch time, the
node bypasses the inference and publishes a constant alpha = 1.0,
which produces the fuzzy-OFF baseline experimental condition described
in Section 3.4.3 and Chapter 5.

This node corresponds to:
  - Chapter 3, Section 3.7 (Mamdani inference)
  - Chapter 4, Section 4.4 (implementation details)

The Mamdani inference structure is described in Section 3.7.1, the
input membership functions in Section 3.7.2, the output membership
functions in Section 3.7.3, the rule base in Section 3.7.4, and the
centroid defuzzification in Section 3.7.5 (equation 3.11).  The full
static input-output characteristic produced by this controller is
shown in Figure 3.8 of Section 3.7.6.

The membership-function parameter values used below are the final
values arrived at after the three tuning iterations documented in
Section 4.4.2.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def triangular(x, a, b, c):
    """Triangular membership function tri(x; a, b, c) with vertices at
    a, b, and c.  Equation form used for the MEDIUM input and the
    MEDIUM output sets (Sections 3.7.2 and 3.7.3)."""
    if x <= a or x >= c:
        return 0.0
    elif x == b:
        return 1.0
    elif x < b:
        return (x - a) / (b - a)
    else:
        return (c - x) / (c - b)


def trapezoidal(x, a, b, c, d):
    """Trapezoidal membership function trap(x; a, b, c, d) with corners
    at a, b, c, and d.  Used for the LOW and HIGH input sets and the
    SLOW and FAST output sets (Sections 3.7.2 and 3.7.3)."""
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:
        return (d - x) / (d - c)


class FuzzyAlphaNode(Node):
    def __init__(self):
        super().__init__('fuzzy_alpha_node')

        self.declare_parameter('input_topic', '/vibration_feature')
        self.declare_parameter('output_topic', '/alpha')
        # fuzzy_enabled switches between the fuzzy-ON and fuzzy-OFF
        # experimental conditions (Chapter 5) without any change to
        # the rest of the pipeline.
        self.declare_parameter('fuzzy_enabled', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.fuzzy_enabled = self.get_parameter('fuzzy_enabled').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            Float32,
            input_topic,
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(Float32, output_topic, 10)

        self.get_logger().info(
            f'Fuzzy alpha node started | input_topic={input_topic}, '
            f'output_topic={output_topic}, fuzzy_enabled={self.fuzzy_enabled}'
        )

    # ---------------------------------------------------------------
    # Input membership functions over the vibration feature v (units of g).
    # Parameter values: see Section 3.7.2 (equations 3.5-3.7).
    # ---------------------------------------------------------------

    def mu_vibration_low(self, v):
        # trap(v; 0.000, 0.000, 0.030, 0.045) -- equation (3.X)
        return trapezoidal(v, 0.000, 0.000, 0.030, 0.045)

    def mu_vibration_medium(self, v):
        # tri(v; 0.035, 0.055, 0.085) -- equation (3.X)
        return triangular(v, 0.035, 0.055, 0.085)

    def mu_vibration_high(self, v):
        # trap(v; 0.070, 0.090, 0.200, 0.200) -- equation (3.7).
        # The explicit saturation for v >= 0.2 is the input clipping
        # described at the start of Section 3.7.2: values above 0.2 are
        # treated as fully HIGH, guaranteeing at least one rule fires
        # under any input and preventing the fall-back-to-0.4 branch
        # of the defuzzification (Section 3.7.5) from ever being taken.
        if v >= 0.2:
            return 1.0
        return trapezoidal(v, 0.070, 0.090, 0.20, 0.20)

    # ---------------------------------------------------------------
    # Output membership functions over the scaling factor alpha
    # (dimensionless, in [0.4, 1.0]).
    # Parameter values: see Section 3.7.3 (equations 3.8-3.10).
    # ---------------------------------------------------------------

    def mu_speed_slow(self, s):
        # trap(alpha; 0.40, 0.40, 0.50, 0.65) -- equation (3.8)
        return trapezoidal(s, 0.40, 0.40, 0.50, 0.65)

    def mu_speed_medium(self, s):
        # tri(alpha; 0.55, 0.72, 0.85) -- equation (3.9)
        return triangular(s, 0.55, 0.72, 0.85)

    def mu_speed_fast(self, s):
        # trap(alpha; 0.75, 0.90, 1.00, 1.00) -- equation (3.10)
        return trapezoidal(s, 0.75, 0.90, 1.00, 1.00)

    def fuzzy_controller(self, vibration_input):
        """Full Mamdani inference: fuzzification -> rule evaluation ->
        clipping -> aggregation -> centroid defuzzification.
        See Section 3.7 for the conceptual description and Section
        4.4.1 for the numerical implementation."""

        # --- Step 1: Fuzzification (Section 3.7.1) ---
        # Evaluate the three input membership functions at the current
        # vibration value.  These are also the firing strengths of the
        # three rules, since each rule has a single antecedent.
        mu_low = self.mu_vibration_low(vibration_input)
        mu_medium = self.mu_vibration_medium(vibration_input)
        mu_high = self.mu_vibration_high(vibration_input)

        # --- Step 2: Rule evaluation (Section 3.7.4, Table 3.3) ---
        # Rule 1: IF v is LOW    THEN alpha is FAST
        # Rule 2: IF v is MEDIUM THEN alpha is MEDIUM
        # Rule 3: IF v is HIGH   THEN alpha is SLOW
        rule_fast = mu_low
        rule_medium = mu_medium
        rule_slow = mu_high

        # Sample the output universe [0.4, 1.0] at 1000 uniform points
        # for the numerical integration of the centroid.  The choice
        # of 1000 points is discussed in Section 4.4.1; doubling to
        # 2000 produces no detectable difference in the output, halving
        # to 500 introduces visible discretization artefacts.
        s_values = np.linspace(0.4, 1.0, 1000)

        # --- Step 3: Clipping (Section 3.7.4) ---
        # Each rule's consequent output set is clipped pointwise at
        # the firing strength of that rule, using the min operator.
        # The resulting clipped consequent set is what that rule
        # contributes to the aggregated output set.  See Section
        # 4.4.1 for the implementation pattern.
        slow_clipped = np.array([min(rule_slow, self.mu_speed_slow(s)) for s in s_values])
        medium_clipped = np.array([min(rule_medium, self.mu_speed_medium(s)) for s in s_values])
        fast_clipped = np.array([min(rule_fast, self.mu_speed_fast(s)) for s in s_values])

        # --- Step 4: Aggregation (Section 3.7.4) ---
        # The three clipped sets are combined into a single aggregated
        # output set by elementwise maximum, which is the standard
        # aggregation operator for Mamdani inference.
        aggregated = np.maximum.reduce([slow_clipped, medium_clipped, fast_clipped])

        # --- Step 5: Defuzzification (Section 3.7.5) ---
        # Discrete Riemann-sum approximation of the centroid integral
        # of equation (3.11) in Section 3.7.5.  The sample spacing
        # delta_s is the same in numerator and denominator and therefore
        # cancels, so it does not appear explicitly here.  See the
        # numerical implementation discussion in Section 4.4.1.
        if np.sum(aggregated) == 0:
            # Robustness guard discussed at the end of Section 3.7.5.
            # Under the saturation in mu_vibration_high above, this
            # branch is unreachable at runtime; it is retained for
            # safety against any future change to the membership
            # functions that would leave a gap in the input coverage.
            alpha = 0.4
        else:
            alpha = np.sum(s_values * aggregated) / np.sum(aggregated)

        return float(alpha)

    def listener_callback(self, msg):
        vibration_value = msg.data

        if self.fuzzy_enabled:
            alpha = self.fuzzy_controller(vibration_value)
        else:
            # Fuzzy-OFF baseline condition.  Section 3.4.3 (topic
            # graph) and Chapter 5 (experimental design).  With
            # alpha = 1.0 the downstream cmd_vel_fusion_node passes
            # the operator's command through unchanged.
            alpha = 1.0

        out_msg = Float32()
        out_msg.data = alpha
        self.publisher.publish(out_msg)

        self.get_logger().info(
            f'Vibration: {vibration_value:.4f} | Alpha: {alpha:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyAlphaNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()