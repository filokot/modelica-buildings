within Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses;
model TimeSeriesBuilding "Building model from time series"
  import Buildings;
  extends Buildings.DistrictEnergySystem.Loads.BaseClasses.PartialBuilding(heaLoaTyp={Buildings.DistrictEnergySystem.Loads.Types.ModelType.ODE});
  Buildings.Controls.OBC.CDL.Continuous.Sources.Constant maxTSet(k=24)
    "Maximum temperature setpoint"
    annotation (Placement(transformation(extent={{-260,-62},{-240,-42}})));
  Buildings.Controls.OBC.UnitConversions.From_degC from_degC1
    annotation (Placement(transformation(extent={{-220,-62},{-200,-42}})));
  Modelica.Blocks.Sources.CombiTimeTable loa(
    tableOnFile=true,
    columns={2,3},
    tableName="csv",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/DistrictEnergySystem/Loads/Examples/Ressources/Loads.csv"),
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments) "Reader for test.csv"
    annotation (Placement(transformation(extent={{20,-8},{40,12}})));

  Buildings.HeatTransfer.Sources.PrescribedTemperature prescribedTemperature
    annotation (Placement(transformation(extent={{-240,-110},{-260,-90}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.SimplifiedBuildingODE simplifiedBuildingODE1(
    TOutHea_nominal=268.15,
    TIndHea_nominal=293.15,
    Q_flowHea_nominal=Q_flowHea_nominal[1],
    Q_flow_nominal=Q_flowCoo_nominal[1]) annotation (Placement(transformation(extent={{-200,-110},{-220,-90}})));
equation
  connect(maxTSet.y, from_degC1.u)
    annotation (Line(points={{-239,-52},{-222,-52}},   color={0,0,127}));
  connect(Q_flowHea.y, Q_flowHeaAct) annotation (Line(points={{281,210},{310,210}}, color={0,0,127}));
  connect(prescribedTemperature.T, simplifiedBuildingODE1.TInd)
    annotation (Line(points={{-238,-100},{-221,-100}}, color={0,0,127}));
  connect(weaBus.TDryBul, simplifiedBuildingODE1.TOut) annotation (Line(
      points={{1,300},{1,-92},{-198,-92}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(loa.y[1], simplifiedBuildingODE1.Q_flowReq)
    annotation (Line(points={{41,2},{80,2},{80,-102},{-198,-102},{-198,-103}}, color={0,0,127}));
  connect(from_degC1.y, simplifiedBuildingODE1.TSet)
    annotation (Line(points={{-199,-52},{-180,-52},{-180,-97.2},{-198,-97.2}}, color={0,0,127}));
  connect(loa.y[2], Q_flowHeaReq[1]) annotation (Line(points={{41,2},{172,2},{172,100},{310,100}}, color={0,0,127}));
  connect(loa.y[1], Q_flowCooReq[1]) annotation (Line(points={{41,2},{172,2},{172,-100},{310,-100}}, color={0,0,127}));
  connect(prescribedTemperature.port, heaPorCoo[1])
    annotation (Line(points={{-260,-100},{-282,-100},{-282,-100},{-300,-100}}, color={191,0,0}));
  connect(Q_flowCoo[1].y, simplifiedBuildingODE1.Q_flowAct)
    annotation (Line(points={{281,-210},{290,-210},{290,-108},{-198,-108}}, color={0,0,127}));
  connect(loa.y[2], buiODE[1].Q_flowReq) annotation (Line(points={{41,2},{80,2},{80,97},{-198,97}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-300,-300},{300,300}})), Icon(
        coordinateSystem(extent={{-100,-100},{100,100}})));
end TimeSeriesBuilding;
