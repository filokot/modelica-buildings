within Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses;
model TimeSeriesBuildingBck "Building model from time series"
  import Buildings;
  extends Buildings.DistrictEnergySystem.Loads.BaseClasses.PartialBuilding(
    final nHeaLoa=1,
    final nCooLoa=1,
    final heaLoaTyp={Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE},
    final cooLoaTyp={Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE});
  Modelica.Blocks.Sources.CombiTimeTable loa(
    tableOnFile=true,
    columns={2,3},
    tableName="csv",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/DistrictEnergySystem/Loads/Examples/Ressources/Loads.csv"),
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments) "Reader for test.csv"
    annotation (Placement(transformation(extent={{20,-8},{40,12}})));

  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1
    annotation (Placement(transformation(extent={{-100,120},{-80,140}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant minTSet(k=20)
    "Minimum temperature setpoint"
    annotation (Placement(transformation(extent={{-140,120},{-120,140}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant maxTSet(k=24)
    "Maximum temperature setpoint"
    annotation (Placement(transformation(extent={{-140,-140},{-120,-120}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC2
    annotation (Placement(transformation(extent={{-100,-140},{-80,-120}})));
equation
  connect(Q_flowHeaVal.y, Q_flowHeaAct) annotation (Line(points={{281,210},{310,210}}, color={0,0,127}));
  connect(TCooLoaODE.T, cooLoaODE.TInd) annotation (Line(points={{-238,-100},{-221,-100}}, color={0,0,127}));
  connect(loa.y[2], Q_flowHeaReq[1]) annotation (Line(points={{41,2},{172,2},{172,100},{310,100}}, color={0,0,127}));
  connect(minTSet.y, from_degC1.u) annotation (Line(points={{-119,130},{-102,130}}, color={0,0,127}));
  connect(from_degC1.y, heaLoaODE[1].TSet)
    annotation (Line(points={{-79,130},{-60,130},{-60,102.8},{-198,102.8}}, color={0,0,127}));
  connect(maxTSet.y, from_degC2.u) annotation (Line(points={{-119,-130},{-102,-130}}, color={0,0,127}));
  connect(from_degC2.y, cooLoaODE[1].TSet)
    annotation (Line(points={{-79,-130},{-60,-130},{-60,-97.2},{-198,-97.2}}, color={0,0,127}));
  connect(loa.y[1], Q_flowCooReq[1]) annotation (Line(points={{41,2},{172,2},{172,-100},{310,-100}}, color={0,0,127}));
  connect(loa.y[1], cooLoaODE[1].Q_flowReq)
    annotation (Line(points={{41,2},{80,2},{80,-103},{-198,-103}}, color={0,0,127}));
  connect(loa.y[2], heaLoaODE[1].Q_flowReq)
    annotation (Line(points={{41,2},{80,2},{80,97},{-198,97}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-300,-300},{300,300}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end TimeSeriesBuildingBck;
