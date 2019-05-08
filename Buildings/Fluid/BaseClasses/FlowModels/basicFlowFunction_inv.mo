within Buildings.Fluid.BaseClasses.FlowModels;
function basicFlowFunction_inv
  "Inverse of flow function that computes the square of flow coefficient"

  input Modelica.SIunits.MassFlowRate m_flow
    "Mass flow rate in design flow direction";
  input Modelica.SIunits.PressureDifference dp
    "Pressure difference between port_a and port_b (= port_a.p - port_b.p)";
  input Modelica.SIunits.MassFlowRate m_flow_turbulent(min=0)
    "Mass flow rate where transition to turbulent flow occurs";
  input Modelica.SIunits.MassFlowRate m_flow_small
    "Minimal value of mass flow rate for guarding against k=(0)/sqrt(dp)";
  input Modelica.SIunits.PressureDifference dp_small
    "Minimal value of pressure drop for guarding against k=m_flow/(0)";
  input Real k_min
    "Minimal value of flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
  input Real k_max
    "Maximal value of flow coefficient, k=m_flow/sqrt(dp), with unit=(kg.m)^(1/2)";
  output Real kSqu
    "Square of flow coefficient";
protected
  Real m_flowNorm = m_flowGuard / m_flow_turbulent
    "Normalised mass flow rate";
  Real m_flowNormSq = m_flowNorm^2
    "Square of normalised mass flow rate";
  Real m_flowGuard = max(abs(m_flow), m_flow_small);
  Real dpGuard = max(abs(dp), dp_small);
algorithm
  kSqu := if noEvent(abs(m_flow) > m_flow_turbulent) then min(k_max^2, max(k_min^2, m_flowGuard^2 / dpGuard))
    else min(k_max^2, max(k_min^2,
      (0.375 + (0.75 - 0.125 * m_flowNormSq) * m_flowNormSq) * m_flow_turbulent^2 / dpGuard * m_flowNorm));
annotation (
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
  -100},{100,100}}), graphics={Line(
  points={{-80,-40},{-80,60},{80,-40},{80,60}},
  color={0,0,255},
  thickness=1), Text(
  extent={{-40,-40},{40,-80}},
  lineColor={0,0,0},
  fillPattern=FillPattern.Sphere,
  fillColor={232,0,0},
  textString="%name")}),
Documentation(info="<html>
<p>
Function that computes the pressure drop of flow elements as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  &Delta;p = sign(m) (m &frasl; k)<sup>2</sup>
</p>
<p>
with regularization near the origin.
Therefore, the flow coefficient is
</p>
<p align=\"center\" style=\"font-style:italic;\">
  k = m &frasl; &radic;<span style=\"text-decoration:overline;\">&nbsp;&Delta;p &nbsp;</span>
</p>
<p>
The input <code>m_flow_turbulent</code> determines the location of the regularization.
</p>
</html>",
revisions="<html>
<ul>
<li>
January 4, 2019, by Michael Wetter:<br/>
Set `Inline=false`.<br/>
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/1070\">#1070</a>.
</li>
<li>
May 1, 2017, by Filip Jorissen:<br/>
Revised implementation such that
<a href=\"modelica://Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow\">
Buildings.Fluid.BaseClasses.FlowModels.basicFlowFunction_m_flow</a>
is C2 continuous.
See <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/725\">#725</a>.
</li>
<li>
January 22, 2016, by Michael Wetter:<br/>
Corrected type declaration of pressure difference.
This is
for <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/404\">#404</a>.
</li>
<li>
July 28, 2015, by Michael Wetter:<br/>
Removed double declaration of <code>smooth(..)</code> and <code>smoothOrder</code>
and changed <code>Inline=true</code> to <code>LateInline=true</code>.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/301\">issue 301</a>.
</li>
<li>
July 15, 2015, by Filip Jorissen:<br/>
New, more efficient implementation based on regularisation using simple polynomial.
Expanded common subexpressions for function inlining to be possible.
Set <code>Inline=true</code> for inlining to occur.
This is for
<a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/279\">#279</a>.
</li>
<li>
August 10, 2011, by Michael Wetter:<br/>
Removed <code>if-then</code> optimization that set <code>dp=0</code> if <code>m_flow=0</code>,
as this causes the derivative to be discontinuous at <code>m_flow=0</code>.
</li>
<li>
August 4, 2011, by Michael Wetter:<br/>
Removed option to use a linear function. The linear implementation is now done
in models that call this function. With the previous implementation,
the symbolic processor may not rearrange the equations, which can lead
to coupled equations instead of an explicit solution.
</li>
<li>
April 13, 2009, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));
end basicFlowFunction_inv;