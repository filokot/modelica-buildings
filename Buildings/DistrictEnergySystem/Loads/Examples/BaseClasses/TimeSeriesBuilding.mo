within Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses;
model TimeSeriesBuilding "Building model from time series"
  import Buildings;
  extends Buildings.DistrictEnergySystem.Loads.BaseClasses.PartialBuilding;
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant maxTSet(k=24)
    "Maximum temperature setpoint"
    annotation (Placement(transformation(extent={{-140,-70},{-120,-50}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1
    annotation (Placement(transformation(extent={{-100,-70},{-80,-50}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
    tableOnFile=true,
    columns={2,3},
    tableName="csv",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/DistrictEnergySystem/Loads/Examples/Ressources/Loads.csv"),
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments)
                     "Reader for test.csv"
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));

  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
    annotation (Placement(transformation(extent={{-56,-70},{-36,-50}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant minTSet(k=20)
    "Minimum temperature setpoint"
    annotation (Placement(transformation(extent={{-140,50},{-120,70}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC2
    annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature2
    annotation (Placement(transformation(extent={{-56,50},{-36,70}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDHea(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    reverseAction=false,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    Ti=120) "PID controller for heating"
    annotation (Placement(transformation(extent={{100,60},{120,80}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai2(k=1/Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={70,70})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=1/Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={110,28})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=heaPorHea.Q_flow)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDCoo(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    Ti=120,
    reverseAction=true) "PID controller for cooling"
    annotation (Placement(transformation(extent={{100,-40},{120,-20}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai3(k=1/Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={70,-30})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai4(k=1/Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={110,-70})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=heaPorCoo.Q_flow)
    annotation (Placement(transformation(extent={{40,-110},{60,-90}})));
equation
  connect(maxTSet.y, from_degC1.u)
    annotation (Line(points={{-119,-60},{-102,-60}},   color={0,0,127}));
  connect(from_degC1.y, prescribedTemperature.T)
    annotation (Line(points={{-79,-60},{-58,-60}}, color={0,0,127}));
  connect(prescribedTemperature.port, heaPorCoo) annotation (Line(points={{-36,
          -60},{-20,-60},{-20,-100},{-300,-100}}, color={191,0,0}));
  connect(minTSet.y, from_degC2.u)
    annotation (Line(points={{-119,60},{-102,60}}, color={0,0,127}));
  connect(from_degC2.y, prescribedTemperature2.T)
    annotation (Line(points={{-79,60},{-58,60}}, color={0,0,127}));
  connect(prescribedTemperature2.port, heaPorHea) annotation (Line(points={{-36,
          60},{-20,60},{-20,100},{-300,100}}, color={191,0,0}));
  connect(conPIDHea.u_s, gai2.y)
    annotation (Line(points={{98,70},{81,70}}, color={0,0,127}));
  connect(conPIDHea.u_m, gai1.y)
    annotation (Line(points={{110,58},{110,39}}, color={0,0,127}));
  connect(combiTimeTable.y[2], gai2.u)
    annotation (Line(points={{1,0},{28,0},{28,70},{58,70}}, color={0,0,127}));
  connect(realExpression.y, gai1.u)
    annotation (Line(points={{61,0},{110,0},{110,16}}, color={0,0,127}));
  connect(conPIDHea.y, yHea) annotation (Line(points={{121,70},{212,70},{212,
          100},{310,100}}, color={0,0,127}));
  connect(conPIDCoo.u_s, gai3.y)
    annotation (Line(points={{98,-30},{81,-30}}, color={0,0,127}));
  connect(conPIDCoo.u_m, gai4.y)
    annotation (Line(points={{110,-42},{110,-59}}, color={0,0,127}));
  connect(realExpression2.y, gai4.u) annotation (Line(points={{61,-100},{110,
          -100},{110,-82}}, color={0,0,127}));
  connect(conPIDCoo.y, yCoo) annotation (Line(points={{121,-30},{212,-30},{212,
          -100},{310,-100}}, color={0,0,127}));
  connect(combiTimeTable.y[1], gai3.u) annotation (Line(points={{1,0},{28.5,0},
          {28.5,-30},{58,-30}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-300,-300},{300,300}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end TimeSeriesBuilding;
