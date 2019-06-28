within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingRCMult2Loads "Example illustrating the coupling of a RC building model to a fluid loop"
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

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCOneElementBuilding2Loads
    bui
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-32,30})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1,
    p=300000) "Sink for heating water"
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
        origin={-30,-90})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingMult  couHea(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    TLoa_nominal=fill(273.15 + 20, size(bui.Q_flowHea_nominal, 1)),
    nLoa=2,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    Q_flowLoa_nominal={0.4,0.9} .* bui.Q_flowHea_nominal)
            annotation (Placement(transformation(extent={{0,36},{20,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingMult  couCoo(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    Q_flowLoa_nominal=bui.Q_flowCoo_nominal,
    TLoa_nominal=fill(24 + 273.15, size(bui.Q_flowCoo_nominal, 1)),
    nLoa=1,
    reverseAction=true,
    T_a_nominal=280.15,
    T_b_nominal=285.15)
            annotation (Placement(transformation(extent={{0,-96},{20,-80}})));
equation
  connect(weaDat.weaBus, bui.weaBus)
  annotation (Line(
      points={{90,-20},{50.1,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(supHea.ports[1], couHea.port_a)
  annotation (Line(points={{-22,30},{0,30}},                      color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1])
  annotation (Line(points={{20,30},{90,30}},                    color={0,127,255}));
  connect(supCoo.ports[1], couCoo.port_a)
  annotation (Line(points={{-20,-90},{0,-90}},                        color={0,127,255}));
  connect(couCoo.port_b, sinCoo.ports[1])
  annotation (Line(points={{20,-90},{90,-90}},                      color={0,127,255}));
  connect(bui.heaPorHea, couHea.heaPorLoa)
  annotation (Line(points={{40,-23},{10,-23},{10,20}}, color={191,0,0}));
  connect(bui.Q_flowHeaReq, couHea.Q_flowLoaReq)
    annotation (Line(points={{61,-27},{80,-27},{80,0},{-10,0},{-10,22},{-2,22}},     color={0,0,127}));
  connect(couHea.m_flowReq, supHea.m_flow_in)
    annotation (Line(points={{21,22},{40,22},{40,60},{-60,60},{-60,38},{-44,38}},     color={0,0,127}));
  connect(bui.Q_flowCooReq, couCoo.Q_flowLoaReq)
    annotation (Line(points={{61,-33},{80,-33},{80,-60},{-10,-60},{-10,-82},{-2,-82}},     color={0,0,127}));
  connect(couCoo.heaPorLoa, bui.heaPorCoo) annotation (Line(points={{10,-80},{10,-37},{40,-37}}, color={191,0,0}));
  connect(couCoo.m_flowReq, supCoo.m_flow_in)
    annotation (Line(points={{21,-82},{40,-82},{40,-120},{-60,-120},{-60,-82},{-42,-82}},     color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{140,80}})),
    __Dymola_Commands(file="Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingRCMult2Loads.mos"
        "Simulate and plot"));
end CouplingRCMult2Loads;
