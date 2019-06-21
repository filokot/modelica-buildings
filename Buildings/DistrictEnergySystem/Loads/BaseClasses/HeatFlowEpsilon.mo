within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowEpsilon
  "Model for static heat transfer between a circulating fluid and a thermal load, based on effectiveness"
  extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
  Buildings.Controls.OBC.CDL.Interfaces.RealInput UA(
    quantity="ThermalConductance",
    unit="W/K",
    displayUnit="W/K",
    min=0)
    "Thermal conductance"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,70}),
                         iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80})));
  Real eps "Heat exchange effectiveness";
  Modelica.SIunits.Temperature port_a_T
  "Temperature of ideally mixed fluid at component side of port_a";
  Modelica.SIunits.Temperature port_b_T
  "Temperature of ideally mixed fluid at component side of port_b";
  Modelica.SIunits.Temperature TInl
  "Temperature of ideally mixed fluid at component side of inlet port";
  Modelica.SIunits.SpecificHeatCapacity cpInl
  "Specific heat capacity of ideally mixed fluid at component side of inlet port";
public
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPor
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
                iconTransformation(extent={{-10,-90},{10,-70}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=heaPor.Q_flow)
    annotation (Placement(transformation(extent={{-40,-2},{-20,18}})));
protected
  Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation staTwoPorConEqu(
    redeclare final package Medium = Medium,
    final simplify_mWat_flow = true,
    final prescribedHeatFlowRate=false,
    final use_C_flow=false,
    final use_mWat_flow=false,
    final m_flow_nominal=m_flow_nominal)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  // Parameters for inverseXRegularized.
  // These are assigned here for efficiency reason.
  // Otherwise, they would need to be computed each time
  // the function is invocated.
  final parameter Real deltaReg = m_flow_small/1E3
    "Smoothing region for inverseXRegularized";
  final parameter Real deltaInvReg = 1/deltaReg
    "Inverse value of delta for inverseXRegularized";
  final parameter Real aReg = -15*deltaInvReg
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real bReg = 119*deltaInvReg^2
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real cReg = -361*deltaInvReg^3
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real dReg = 534*deltaInvReg^4
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real eReg = -380*deltaInvReg^5
    "Polynomial coefficient for inverseXRegularized";
  final parameter Real fReg = 104*deltaInvReg^6
    "Polynomial coefficient for inverseXRegularized";
  Real m_flow_inv(unit="s/kg") "Regularization of 1/m_flow";
  Modelica.SIunits.MassFlowRate m_flow_pos;
equation
  eps = 1 - exp(-UA * m_flow_inv / cpInl);
  heaPor.Q_flow = eps * m_flow_pos * cpInl * (heaPor.T - TInl);
  m_flow_pos = Modelica.Fluid.Utilities.regStep(
    x=m_flow,
    y1=m_flow,
    y2=-m_flow,
    x_small=m_flow_small);
  m_flow_inv = Buildings.Utilities.Math.Functions.inverseXRegularized(
     x=m_flow_pos,
     delta=deltaReg, deltaInv=deltaInvReg, a=aReg, b=bReg, c=cReg, d=dReg, e=eReg, f=fReg);
  if allowFlowReversal then
    TInl = Modelica.Fluid.Utilities.regStep(
      x=m_flow,
      y1=port_a_T,
      y2=port_b_T,
      x_small=m_flow_small);
  else
    TInl = port_a_T;
  end if;
  port_a_T = Medium.temperature(
    Medium.setState_phX(
      p=port_a.p,
      h=inStream(port_a.h_outflow),
      X=inStream(port_a.Xi_outflow)));
  port_b_T = Medium.temperature(
    Medium.setState_phX(
      p=port_b.p,
      h=inStream(port_b.h_outflow),
      X=inStream(port_b.Xi_outflow)));
  cpInl = Medium.specificHeatCapacityCp(
    Medium.setState_pTX(
      p=port_a.p,
      T=TInl));
  connect(port_a, staTwoPorConEqu.port_a) annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(staTwoPorConEqu.port_b, port_b) annotation (Line(points={{10,0},{100,0}}, color={0,127,255}));

  connect(realExpression.y, staTwoPorConEqu.Q_flow) annotation (Line(points={{-19,8},{-12,8}}, color={0,0,127}));
  annotation (
  defaultComponentName="heaFloEps",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={192,192,192},
          fillPattern=FillPattern.Forward)}),                    Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatFlowEpsilon;
