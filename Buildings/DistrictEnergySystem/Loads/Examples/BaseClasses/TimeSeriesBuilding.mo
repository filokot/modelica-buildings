within Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses;
model TimeSeriesBuilding "Building model from time series"
  import Buildings;
  extends Buildings.DistrictEnergySystem.Loads.BaseClasses.PartialBuilding;
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant maxTSet(k=24)
    "Maximum temperature setpoint"
    annotation (Placement(transformation(extent={{-140,-70},{-120,-50}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1
    annotation (Placement(transformation(extent={{-100,-70},{-80,-50}})));
  Modelica.Blocks.Sources.CombiTimeTable loa(
    tableOnFile=true,
    columns={2,3},
    tableName="csv",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/DistrictEnergySystem/Loads/Examples/Ressources/Loads.csv"),
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments) "Reader for test.csv"
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
    annotation (Placement(transformation(extent={{150,58},{170,78}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai2(k=1/Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={120,68})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=1/Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={160,26})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=heaPorHea.Q_flow)
    annotation (Placement(transformation(extent={{90,-12},{110,8}})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDCoo(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    Ti=120,
    reverseAction=true) "PID controller for cooling"
    annotation (Placement(transformation(extent={{150,-42},{170,-22}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai3(k=1/Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={120,-32})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai4(k=1/Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={160,-72})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=heaPorCoo.Q_flow)
    annotation (Placement(transformation(extent={{90,-112},{110,-92}})));
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
  connect(conPIDHea.u_s,gai2. y)
    annotation (Line(points={{148,68},{131,68}},
                                               color={0,0,127}));
  connect(conPIDHea.u_m,gai1. y)
    annotation (Line(points={{160,56},{160,37}}, color={0,0,127}));
  connect(realExpression1.y, gai1.u) annotation (Line(points={{111,-2},{160,-2},{160,14}}, color={0,0,127}));
  connect(conPIDHea.y, yHea) annotation (Line(points={{171,68},{262,68},{262,100},{310,100}},
                           color={0,0,127}));
  connect(conPIDCoo.u_s,gai3. y)
    annotation (Line(points={{148,-32},{131,-32}},
                                                 color={0,0,127}));
  connect(conPIDCoo.u_m,gai4. y)
    annotation (Line(points={{160,-44},{160,-61}}, color={0,0,127}));
  connect(realExpression3.y,gai4. u) annotation (Line(points={{111,-102},{160,-102},{160,-84}},
                            color={0,0,127}));
  connect(conPIDCoo.y, yCoo) annotation (Line(points={{171,-32},{262,-32},{262,-100},{310,-100}},
                             color={0,0,127}));
  connect(loa.y[2], gai2.u) annotation (Line(points={{1,0},{40,0},{40,68},{108,68}}, color={0,0,127}));
  connect(loa.y[1], gai3.u) annotation (Line(points={{1,0},{40.5,0},{40.5,-32},{108,-32}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-300,-300},{300,300}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end TimeSeriesBuilding;
