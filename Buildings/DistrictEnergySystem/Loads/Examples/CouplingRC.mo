within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingRC "Example illustrating the coupling of a RC building model to a fluid loop"
  import Buildings;
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{110,-30},{90,-10}})));

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCOneElementBuilding
    bui(Q_flowHea_nominal=500, Q_flowCoo_nominal=1000)
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-48,30})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for heating water"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,30})));
  Buildings.Fluid.Sources.FixedBoundary sinCoo(
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for chilled water" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,-90})));
  Buildings.Fluid.Sources.MassFlowSource_T supCoo(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    nPorts=1)             "Supply for chilled water" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,-90})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloHea(k=couHea.m_flow_nominal)
    annotation (Placement(transformation(extent={{-90,28},{-70,48}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloCoo(k=couCoo.m_flow_nominal)
    annotation (Placement(transformation(extent={{-88,-92},{-68,-72}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couHea(
    redeclare package Medium = Medium,
    Q_flow_nominal=bui.Q_flowHea_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{-20,40},{0,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couCoo(
    redeclare package Medium = Medium,
    Q_flow_nominal=bui.Q_flowCoo_nominal,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    TLoa_nominal=297.15) annotation (Placement(transformation(extent={{-20,-100},{0,-80}})));
equation
  connect(weaDat.weaBus, bui.weaBus) annotation (Line(
      points={{90,-20},{50.1,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(mFloHea.y, supHea.m_flow_in)
    annotation (Line(points={{-69,38},{-60,38}}, color={0,0,127}));
  connect(supCoo.m_flow_in, mFloCoo.y)
    annotation (Line(points={{-56,-82},{-67,-82}}, color={0,0,127}));

  connect(supHea.ports[1], couHea.port_a) annotation (Line(points={{-38,30},{-20,30}}, color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1]) annotation (Line(points={{0,30},{90,30}}, color={0,127,255}));
  connect(couHea.heaPorLoa, bui.heaPorHea) annotation (Line(points={{-10,20},{-10,-23},{40,-23}}, color={191,0,0}));
  connect(supCoo.ports[1], couCoo.port_a) annotation (Line(points={{-34,-90},{-20,-90}}, color={0,127,255}));
  connect(couCoo.port_b, sinCoo.ports[1]) annotation (Line(points={{0,-90},{90,-90}}, color={0,127,255}));
  connect(couCoo.heaPorLoa, bui.heaPorCoo) annotation (Line(points={{-10,-80},{-10,-37},{40,-37}}, color={191,0,0}));
  connect(bui.yCoo, mFloCoo.u)
    annotation (Line(points={{61,-37},{80,-37},{80,-58},{-100,-58},{-100,-82},{-90,-82}}, color={0,0,127}));
  connect(bui.yHea, mFloHea.u)
    annotation (Line(points={{61,-23},{80,-23},{80,0},{-100,0},{-100,38},{-92,38}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{120,80}})),
    __Dymola_Commands(file="Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingRC.mos"
        "Simulate and plot"),
    Icon(coordinateSystem(extent={{-140,-140},{120,80}})));
end CouplingRC;
