within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model HeatFlowEpsilon1
  "Model for static heat transfer between a circulating fluid and a thermal load, based on effectiveness"
  extends Buildings.Fluid.Interfaces.StaticTwoPortConservationEquation(
    final simplify_mWat_flow = true,
    final prescribedHeatFlowRate=false,
    final use_C_flow=false,
    final use_mWat_flow=false,
    show_T=true,
    Q_flow=heaPor.Q_flow);

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
  Medium.Temperature port_a_T
  "Temperature of ideally mixed fluid at component side of port_a";
  Medium.Temperature port_b_T
  "Temperature of ideally mixed fluid at component side of port_b";
  Medium.Temperature TInl
  "Temperature of ideally mixed fluid at component side of inlet port";
  Medium.SpecificHeatCapacity cpInl
  "Specific heat capacity of ideally mixed fluid at component side of inlet port";
public
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPor
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}}),
                iconTransformation(extent={{-10,-90},{10,-70}})));
protected
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
     delta=deltaReg, deltaInv=deltaInvReg,
     a=aReg, b=bReg, c=cReg, d=dReg, e=eReg, f=fReg);
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
  annotation (
  defaultComponentName="heaFloEps",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={192,192,192},
          fillPattern=FillPattern.Forward)}),                    Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatFlowEpsilon1;
