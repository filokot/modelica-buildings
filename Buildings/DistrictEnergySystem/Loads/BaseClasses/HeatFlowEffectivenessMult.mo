within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowEffectivenessMult
  "Model for static heat transfer between a circulating fluid and a thermal load, based on effectiveness"
  extends Buildings.Fluid.Interfaces.StaticTwoPortHeatMassExchanger(
    final sensibleOnly=true,
    final prescribedHeatFlowRate=false,
    dp_nominal=0,
    Q_flow=sum(heaPor.Q_flow),
    mWat_flow=0,
    heaInp(y=Q_flow));
  parameter Integer nLoa;
  Buildings.Controls.OBC.CDL.Interfaces.RealInput UA[nLoa](
    each quantity="ThermalConductance", each unit="W/K", each min=0) "Thermal conductance"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80}),iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80})));
  Modelica.SIunits.Temperature port_a_T
  "Temperature of ideally mixed fluid at component side of port_a";
  Modelica.SIunits.Temperature port_b_T
  "Temperature of ideally mixed fluid at component side of port_b";
  Modelica.SIunits.Temperature TInl
  "Temperature of ideally mixed fluid at component side of inlet port";
  Modelica.SIunits.SpecificHeatCapacity cpInl
  "Specific heat capacity of ideally mixed fluid at component side of inlet port";
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPor[nLoa]
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
                iconTransformation(extent={{-10,-90},{10,-70}})));
  Buildings.Controls.OBC.CDL.Interfaces.RealInput m_flowLoa[nLoa](
    each quantity="MassFlowRate") "Mass flow rate" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-70}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-80})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.EffectivenessDirect effDir[nLoa]
    annotation (Placement(transformation(extent={{-54,62},{-34,82}})));
  Modelica.Blocks.Sources.RealExpression TInlVal[nLoa](y=fill(TInl, nLoa))
    "Fluid inlet temperature"
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  Modelica.Blocks.Sources.RealExpression cpInlVal[nLoa](y=fill(cpInl, nLoa))
    "Fluid inlet specific heat capacity"
    annotation (Placement(transformation(extent={{-100,26},{-80,46}})));
  Modelica.Blocks.Sources.RealExpression heaPorTVal[nLoa](y=heaPor.T)
    "Temperature of the load"
    annotation (Placement(transformation(extent={{-100,12},{-80,32}})));
  Modelica.Blocks.Sources.RealExpression m_flowActVal[nLoa](y=m_flowAct)
    "Actual mass flow rate"
    annotation (Placement(transformation(extent={{-100,54},{-80,74}})));
  // Parameters for inverseXRegularized.
  // These are assigned here for efficiency reason.
  // Otherwise, they would need to be computed each time
  // the function is invocated.
protected
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
  Modelica.SIunits.MassFlowRate m_flowAct[nLoa];
equation
  m_flowAct = Modelica.Fluid.Utilities.regStep(
    sum(m_flowLoa) - 2 * m_flow_small,
    m_flowLoa / sum(m_flowLoa) * m_flow,
    fill(0, nLoa),
    m_flow_small);
  heaPor.Q_flow = effDir.Q_flow;
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
  connect(UA, effDir.UA) annotation (Line(points={{-120,80},{-56,80}}, color={0,0,127}));
  connect(heaPorTVal.y, effDir.TLoad) annotation (Line(points={{-79,22},{-70,22},{-70,64},{-56,64}}, color={0,0,127}));
  connect(cpInlVal.y, effDir.cpInl) annotation (Line(points={{-79,36},{-72,36},{-72,68},{-56,68}}, color={0,0,127}));
  connect(TInlVal.y, effDir.TInl) annotation (Line(points={{-79,50},{-74,50},{-74,72},{-56,72}}, color={0,0,127}));
  connect(m_flowActVal.y, effDir.m_flow)
    annotation (Line(points={{-79,64},{-76,64},{-76,76},{-56,76}}, color={0,0,127}));
  annotation (
  defaultComponentName="heaFloEps",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={192,192,192},
          fillPattern=FillPattern.Forward)}),                    Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatFlowEffectivenessMult;
