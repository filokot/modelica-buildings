within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model EffectivenessDirect
  extends Modelica.Blocks.Icons.Block;
  parameter Real m_flow_small = 1E-2;
  Real eps "Heat exchanger effectiveness";
  Controls.OBC.CDL.Interfaces.RealInput UA(
    quantity="ThermalConductance",
    unit="W/K",
    min=0) "Thermal conductance"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,90}),
                         iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,80})));
  Controls.OBC.CDL.Interfaces.RealInput TInl(
    quantity="ThermodynamicTemperature", displayUnit="degC") "Fluid inlet temperature" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,10}),  iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0})));
  Controls.OBC.CDL.Interfaces.RealInput m_flow(quantity="MassFlowRate") "Fluid mass flow rate" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,50}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Controls.OBC.CDL.Interfaces.RealInput TLoad(
    quantity="ThermodynamicTemperature", displayUnit="degC") "Temperature of the load" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-90}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-80})));

  Controls.OBC.CDL.Interfaces.RealInput cpInl(quantity="SpecificHeatCapacity")
    "Fluid inlet specific heat capacity"
    annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-30}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-40})));
  Controls.OBC.CDL.Interfaces.RealOutput Q_flow(quantity="HeatFlowRate") "Heat flow rate"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
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
  Real m_flow_inv(unit="s/kg") "Regularization of 1/m_flow";
  Real m_flow_pos;
equation
  eps = 1 - exp(-UA * m_flow_inv / cpInl);
  Q_flow = eps * m_flow_pos * cpInl * (TLoad - TInl);
  m_flow_pos = Buildings.Utilities.Math.Functions.regStep(
    m_flow,
    m_flow,
    -m_flow,
    m_flow_small);
  m_flow_inv = Buildings.Utilities.Math.Functions.inverseXRegularized(
     x=m_flow_pos,
     delta=deltaReg, deltaInv=deltaInvReg, a=aReg, b=bReg, c=cReg, d=dReg, e=eReg, f=fReg);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end EffectivenessDirect;
