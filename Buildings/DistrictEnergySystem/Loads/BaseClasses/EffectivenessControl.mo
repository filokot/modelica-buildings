within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model EffectivenessControl
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal;
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal;
  Controls.OBC.CDL.Continuous.Gain           gai1(k=-1/Q_flow_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-40,28})));
  Controls.OBC.CDL.Continuous.Gain           gai2(k=1/Q_flow_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-74,60})));
  Controls.OBC.CDL.Continuous.LimPID conPID(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    k=1,
    Ti=10,
    reverseAction=reverseAction)
          "PID controller" annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
  Controls.OBC.CDL.Continuous.Gain           mFloReq(k=m_flow_nominal)
    annotation (Placement(transformation(extent={{-8,50},{12,70}})));
  EffectivenessDirect effectivenessDirect annotation (Placement(transformation(extent={{64,46},{84,66}})));
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
  Controls.OBC.CDL.Interfaces.RealInput TLoad(quantity="ThermodynamicTemperature", displayUnit="degC")
                                                             "Temperature of the load" annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-70}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-80})));
  Controls.OBC.CDL.Interfaces.RealInput Q_flow(quantity="HeatFlowRate") "Heat flow rate"       annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,50}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Controls.OBC.CDL.Interfaces.RealOutput m_flow(quantity="MassFlowRate") "Mass flow rate"
    annotation (Placement(transformation(extent={{100,30},{120,50}}), iconTransformation(extent={{100,40},{120,60}})));
  Controls.OBC.CDL.Interfaces.RealOutput Q_flowAct(quantity="HeatFlowRate") "Heat flow rate" annotation (Placement(
        transformation(extent={{100,-50},{120,-30}}), iconTransformation(extent={{100,-60},{120,-40}})));
  parameter Boolean reverseAction=false
    "Set to true for throttling the water flow rate through a cooling coil controller";
equation
  connect(conPID.y, mFloReq.u) annotation (Line(points={{-29,60},{-24,60},{-20,60},{-10,60}},
                                                                            color={0,0,127}));
  connect(gai2.y, conPID.u_s) annotation (Line(points={{-63,60},{-52,60}}, color={0,0,127}));
  connect(conPID.u_m, gai1.y) annotation (Line(points={{-40,48},{-40,39}}, color={0,0,127}));
  connect(UA, effectivenessDirect.UA) annotation (Line(points={{-120,90},{50,90},{50,64},{62,64}}, color={0,0,127}));
  connect(effectivenessDirect.Q_flow, gai1.u)
    annotation (Line(points={{85,56},{88,56},{88,0},{-40,0},{-40,16}},     color={0,0,127}));
  connect(Q_flow, gai2.u) annotation (Line(points={{-120,50},{-94,50},{-94,60},{-86,60}}, color={0,0,127}));
  connect(mFloReq.y, effectivenessDirect.m_flow) annotation (Line(points={{13,60},{26,60},{38,60},{62,60}},
                                                                                            color={0,0,127}));
  connect(mFloReq.y, m_flow) annotation (Line(points={{13,60},{40,60},{40,40},{110,40}}, color={0,0,127}));
  connect(cpInl, effectivenessDirect.cpInl)
    annotation (Line(points={{-120,-30},{54,-30},{54,52},{62,52}}, color={0,0,127}));
  connect(TLoad, effectivenessDirect.TLoad)
    annotation (Line(points={{-120,-70},{56,-70},{56,48},{62,48}}, color={0,0,127}));
  connect(effectivenessDirect.Q_flow, Q_flowAct)
    annotation (Line(points={{85,56},{88,56},{88,-40},{110,-40}}, color={0,0,127}));
  connect(TInl, effectivenessDirect.TInl)
    annotation (Line(points={{-120,10},{-80,10},{-80,-10},{52,-10},{52,56},{62,56}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}})),
                                                                 Diagram(coordinateSystem(preserveAspectRatio=false, extent={
            {-100,-80},{100,100}})));
end EffectivenessControl;
