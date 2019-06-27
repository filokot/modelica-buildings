within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model EffectivenessInverse
  extends Modelica.Blocks.Icons.Block;
  Modelica.Blocks.Math.InverseBlockConstraints inv(y1(start=0))
    annotation (Placement(transformation(extent={{-22,24},{18,48}})));
  EffectivenessDirect effectivenessDirect annotation (Placement(transformation(extent={{8,26},{-12,46}})));
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
  Controls.OBC.CDL.Interfaces.RealInput TInl(quantity="ThermodynamicTemperature", displayUnit="degC")
                                                             "Fluid inlet temperature" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,10}),  iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0})));
  Controls.OBC.CDL.Interfaces.RealInput Q_flow_(quantity="HeatFlowRate") "Heat flow rate" annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,50}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Controls.OBC.CDL.Interfaces.RealInput TLoad(quantity="ThermodynamicTemperature", displayUnit="degC")
                                                             "Temperature of the load" annotation (
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
  Controls.OBC.CDL.Interfaces.RealOutput m_flow(quantity="MassFlowRate") "Mass flow rate"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  connect(inv.u2, effectivenessDirect.Q_flow) annotation (Line(points={{-18,36},{-13,36}}, color={0,0,127}));
  connect(inv.y2, effectivenessDirect.m_flow)
    annotation (Line(points={{15,36},{14,36},{14,40},{10,40}}, color={0,0,127}));
  connect(inv.y1, m_flow) annotation (Line(points={{19,36},{60,36},{60,0},{110,0}}, color={0,0,127}));
  connect(Q_flow_, inv.u1) annotation (Line(points={{-120,50},{-72,50},{-72,36},{-24,36}}, color={0,0,127}));
  connect(UA, effectivenessDirect.UA) annotation (Line(points={{-120,90},{-54,90},{-54,44},{10,44}}, color={0,0,127}));
  connect(TInl, effectivenessDirect.TInl)
    annotation (Line(points={{-120,10},{-55,10},{-55,36},{10,36}}, color={0,0,127}));
  connect(cpInl, effectivenessDirect.cpInl)
    annotation (Line(points={{-120,-30},{-55,-30},{-55,32},{10,32}}, color={0,0,127}));
  connect(TLoad, effectivenessDirect.TLoad)
    annotation (Line(points={{-120,-90},{-56,-90},{-56,28},{10,28}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end EffectivenessInverse;
