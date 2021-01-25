#include "spirit_utils/trajectory_visualization_publisher.h"

TrajectoryVisualizationPublisher::TrajectoryVisualizationPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string ground_truth_state_topic;

  spirit_utils::loadROSParam(nh_,"topics/ground_truth_state",ground_truth_state_topic);

  // Setup pubs and subs
  ground_truth_state_pub_ = nh_.advertise<spirit_msgs::StateEstimate>(ground_truth_state_topic,1);

  // loadCSV();

  iterationCount = -1;

}

void TrajectoryVisualizationPublisher::loadCSV() {
  int a = 0;
}

spirit_msgs::StateEstimate TrajectoryVisualizationPublisher::updateStep() {
  spirit_msgs::StateEstimate new_state_est;

  // Hardcode in traj for now

  static const double q1_T[] = {0.701158, 0.7003792716739764, 0.6985824392060491, 0.6958121182245425, 0.6921132168334012, 0.6872219613804207, 0.6815357795403034, 0.6751555773892726, 0.6681324348863569, 0.6603546975196519, 0.6520587907254946, 0.6433571894692421, 0.6342961893219655, 0.6248588427461719, 0.6151685708295205, 0.6053109260570309, 0.5953194442710853, 0.5852176230630453, 0.5750645346139583, 0.564897112815593, 0.5547295655776879, 0.5445783836358062, 0.5344430311817265, 0.5242922207322234, 0.5140904230586663, 0.503770997526151, 0.49332386348068774, 0.4846847137995221, 0.4801345187409375, 0.4829803393649094, 0.5042502506373132, 0.5457296031546269, 0.6053458946050521, 0.6802641896274991, 0.7380948921011908, 0.7732687129404985, 0.7921424787599437, 0.8006818368596973, 0.8017497808557148, 0.799244736460416, 0.7942754536893513, 0.7876139145844949, 0.7800307424979233, 0.7725755918640131, 0.7655681686137941, 0.7592416494545913, 0.7541069644351861, 0.7501822131331052, 0.7472500281115134, 0.7452508823520895, 0.744225346503083, 0.743794644347425, 0.7435440292162546, 0.7432512268165025, 0.7425430832121644, 0.7408172997538892, 0.7380048185567688, 0.733957424824243, 0.728358011534141, 0.7208108701046139, 0.7122921379014129, 0.7037136516934818, 0.6963567519466036, 0.6974770304247422, 0.7145429026485213, 0.753888433162818, 0.8219404732327443, 0.7631636955482126, 0.7075670614991926, 0.6554910818038723, 0.6072619296544324, 0.564994348588784, 0.5274475602852041, 0.4944097757290281, 0.4659082772687727, 0.44346240571441936, 0.4257626965524884, 0.4120543982956844, 0.40209942401412363, 0.39640434625977194, 0.3937273538856703, 0.39316157766333887, 0.3944906099917188, 0.39805118549818236, 0.40407233006099186, 0.41213052640917647, 0.42221974970276804, 0.43464216890678004, 0.4493858030815118, 0.46566863282965365, 0.4832848320102554, 0.5020812576296337, 0.5215603778219557, 0.5409437933504624, 0.5598141522664997, 0.5776310560235383, 0.5926710302724377, 0.60466752671205, 0.6129225714745142, 0.6167266343855909};
  static const double q2_T[] = {1.415082, 1.4171800647694917, 1.421931129676328, 1.4291224300086767, 1.4385413029901275, 1.4505732892505498, 1.4641518537872975, 1.4789703940173202, 1.4948323749014436, 1.5116793052024085, 1.5289602302154315, 1.5464200173704314, 1.5638895460940607, 1.5810402753050765, 1.5975451746168505, 1.6132896994947072, 1.6281145964578607, 1.6415177870737423, 1.6531135746587984, 1.662993917718876, 1.6709761611924063, 1.6764288147419444, 1.6785866101054434, 1.6778279172715713, 1.6738882038306535, 1.6660138605501429, 1.6523764026041634, 1.6324439976583323, 1.6041092850571694, 1.5641008783608545, 1.4997852562475786, 1.4090795620503371, 1.2941016628519948, 1.157879097796342, 1.0760711194969133, 1.056278543436139, 1.079139727392701, 1.126665449612498, 1.191418327300012, 1.26436995740399, 1.3429560338468034, 1.4253898982747963, 1.5098790893145477, 1.5939882463926662, 1.676915930822036, 1.7580381612922895, 1.8358273740199018, 1.9097455721850247, 1.9799527561422168, 2.046145527283471, 2.1068649256176166, 2.161449478855802, 2.2106476995436237, 2.254162613238783, 2.290517679741103, 2.3182381793692093, 2.3388014502688437, 2.3518288749677594, 2.3558710682255417, 2.344202623887087, 2.314629636413395, 2.2628801156463982, 2.18245676767188, 2.0499012286233844, 1.8648448292354205, 1.620908396941032, 1.3120104018599157, 1.373484280467558, 1.4267388549258182, 1.4724961163946924, 1.511456008150651, 1.5421336467169469, 1.567416884489648, 1.5879861682631713, 1.6041760725158518, 1.6151313801076426, 1.6223714828841977, 1.6266346278082686, 1.6282070013879004, 1.626881914381334, 1.6236047762215666, 1.6190135847931135, 1.613290135470549, 1.6063204374360927, 1.5980626321336302, 1.588772530651019, 1.5784725576398237, 1.5670387320459145, 1.5545124731270212, 1.5412900550448292, 1.5274908092372803, 1.5132264055776172, 1.4988418452511414, 1.484737630075193, 1.4711776106765881, 1.4585184486088585, 1.4479555741506125, 1.4395854192943915, 1.4338601064462961, 1.4312385026654715};
  static const double q3_T[] = {0.640798, 0.6393570066391284, 0.6360843369251585, 0.6311153381219674, 0.6245850291344275, 0.6161827955768846, 0.6066264461833398, 0.596108042290958, 0.5847404731343743, 0.5724607498112253, 0.5596296500201716, 0.5464131807057623, 0.532897985132638, 0.5191638029573087, 0.5054101450069665, 0.49174825861931293, 0.47826092840175904, 0.4651403751968399, 0.45259992237523117, 0.4406759253010864, 0.4294779374989407, 0.41931582517362637, 0.41062262668752947, 0.4033132085087405, 0.3975703989240772, 0.39384392084212566, 0.3932753629446458, 0.39638743848216207, 0.4045048710296538, 0.4196543101516685, 0.4492381951203334, 0.4937869477964651, 0.5514014481671607, 0.619611872021851, 0.6683256113360768, 0.6975825680753146, 0.7137341070380938, 0.7225437425997552, 0.7236505146676732, 0.7207501261439494, 0.7149378459358898, 0.7069661003723074, 0.6975092658060763, 0.6878622526930838, 0.6784962488990545, 0.669748712807069, 0.6623533266433285, 0.6566326784931127, 0.6525239311456102, 0.6501459680949597, 0.6500682049003303, 0.6523983430293989, 0.656687345954947, 0.6629239449500504, 0.6714116657299618, 0.6822610746975561, 0.6948536293605582, 0.7091448433064702, 0.7252793195412252, 0.7436187898968297, 0.7639173580372862, 0.7870262781762132, 0.8143320591700947, 0.8550973230946395, 0.9152661039445349, 0.9996549335885109, 1.112988445936333, 1.0598761175699565, 1.0123492880769873, 0.9698180486005492, 0.9317221967620586, 0.8990785300068438, 0.8700822647228238, 0.8443303154598051, 0.8216454734725435, 0.8027358986634652, 0.7865984719561422, 0.7727344960570682, 0.7609714133086432, 0.7515665478949146, 0.7438769061042357, 0.7374251316608524, 0.7320933427828797, 0.7280399411745937, 0.7252769847960375, 0.7235456647229191, 0.7228285293163099, 0.7232665797853587, 0.7248781535171768, 0.7273085389932452, 0.7304884892848966, 0.7344029395692349, 0.738956980109644, 0.7437780389739369, 0.748719483196402, 0.7536080867172329, 0.7579280390626238, 0.7614491033826676, 0.7639168846521736, 0.7650730387879959};
  static const double q4_T[] = {1.376033, 1.3791213591075475, 1.3860684704124935, 1.3965129924387216, 1.410094445385586, 1.4272170741654973, 1.4463308628101341, 1.4669864355495954, 1.4888851688490328, 1.511841837045017, 1.5351412433467968, 1.5584640427028125, 1.5815829677312512, 1.6039986546543001, 1.6253045811756668, 1.6453839918349487, 1.6640207048652051, 1.6804811237375425, 1.6942098478243064, 1.7053437390101465, 1.7135947502275364, 1.7179739711143522, 1.7172022589158533, 1.711749568200014, 1.701080259460127, 1.6837907426582892, 1.6565778304172873, 1.6208881744747519, 1.576012857097942, 1.5205695641763732, 1.447274832553598, 1.3559625406917226, 1.2485775177642446, 1.1276854709697228, 1.0667097741718998, 1.0587930044146754, 1.0840815723618802, 1.124742115809358, 1.1829090211600688, 1.2502009942108188, 1.3241378243830533, 1.403024225292597, 1.4854209001566794, 1.5684828209667385, 1.651191096146331, 1.7328355073407922, 1.811939666755384, 1.8877299974601596, 1.9601943272219657, 2.028963741319188, 2.0925396283437077, 2.1501003389810123, 2.2022557531844473, 2.248617872816288, 2.2875343780408586, 2.317189995604811, 2.338948744328213, 2.3522065860732795, 2.355050963926803, 2.338568007209145, 2.2994804582849158, 2.2320960138633, 2.1278177600217374, 1.953906546939735, 1.7170749048393077, 1.4152843301380797, 1.0471975511966023, 1.1077254701878403, 1.155070924833729, 1.1918531533578984, 1.2205689460520874, 1.2417047210076075, 1.2590961505192502, 1.2736779376972072, 1.286058441078042, 1.2964721936953765, 1.3058995262424626, 1.314676898776426, 1.322964217128125, 1.3308167549007548, 1.3383197197722725, 1.345538803069807, 1.3525281614217028, 1.3594289912922528, 1.366602839258622, 1.374144126195839, 1.3821266729344832, 1.3907095738670516, 1.3999682382381253, 1.4096924003490694, 1.419818284548688, 1.430287362866782, 1.4408710876280773, 1.451275608639204, 1.4613093947363516, 1.4707141590651505, 1.478618925246173, 1.4849226322140032, 1.4892614035088876, 1.4912637456505304};

  static const double x_T[] = {0.0, 4.709561781480408e-05, 0.00016063541735551314, 0.0003430751492534675, 0.0005968604543318853, 0.0009573040113877158, 0.0014025905432124477, 0.0019309345876934957, 0.0025450514187279633, 0.0032819681575467123, 0.004127733018357049, 0.005075996690372445, 0.006131221200850856, 0.007334985139593031, 0.008687279829619964, 0.010175624861662364, 0.0118080588502174, 0.013634365702233744, 0.015679701671376733, 0.017921220450602297, 0.020373536159969077, 0.023098714747135457, 0.026171150652130735, 0.029548493782070195, 0.03325781382569676, 0.03737757532867526, 0.04208281288546001, 0.04727352910962271, 0.0529821857101727, 0.05927771578227156, 0.0664998463892455, 0.07454178567565913, 0.08321428801247685, 0.09229295831933737, 0.10109936177097258, 0.10982904416353438, 0.11853409057297719, 0.1272609458206066, 0.1360219099450205, 0.14480699312266754, 0.15361473273656334, 0.16244521007023874, 0.17130814630486169, 0.1802006240253754, 0.18912126436686244, 0.19807141499055544, 0.20706219286396615, 0.21609163631917827, 0.22515419956102217, 0.2342497582372721, 0.24338515178180847, 0.25255729812951844, 0.2617560432383042, 0.27097872586088445, 0.2802247958538256, 0.2894872743727516, 0.2987556124599631, 0.3080252431317645, 0.3172899550026118, 0.32653607435640875, 0.3357583440316923, 0.34494906944338055, 0.3540967946422238, 0.3631397870245761, 0.37203228423699763, 0.38071844445032077, 0.3891408111131714, 0.3977276527463182, 0.4057157852259043, 0.41316065656152673, 0.42011565221942104, 0.42647163310239966, 0.4324323246180909, 0.4380432057552132, 0.4433246261807553, 0.4481997446145566, 0.45277094422559727, 0.45708646392668373, 0.4611591771057191, 0.4649418942085184, 0.46848565011386273, 0.47183817050430965, 0.47500634080155, 0.4779545521478028, 0.4806862999037841, 0.48324479397702746, 0.48563137616757945, 0.487817778603438, 0.4897871391734717, 0.49159078322308364, 0.49323000494767566, 0.4946872352572306, 0.4959275745670459, 0.49700992662289034, 0.4979357458363021, 0.4986975557412361, 0.49924486203282104, 0.4996431975076077, 0.49989432041240534, 0.5};
  static const double y_T[] = {0.45, 0.4494794944319172, 0.44829964605991085, 0.44651091304807067, 0.4441635280799612, 0.4411490048880653, 0.43772656354044526, 0.4339678907315148, 0.4299167173609125, 0.4255653292773726, 0.4210526720519618, 0.4164450816313107, 0.4117829235883194, 0.40713132656103485, 0.4025803306539816, 0.39817125598571407, 0.3939473682311475, 0.39003157694574586, 0.38653474761977064, 0.3834516256119836, 0.38083680672645476, 0.37886232018152033, 0.3777486512254121, 0.3774141049147636, 0.37794022712535064, 0.3795453314196175, 0.3827287348283199, 0.3873725799872898, 0.3937272229888939, 0.402212720416108, 0.4144991618855643, 0.4303838477654113, 0.44899896868587647, 0.46930454237608904, 0.4873463357939614, 0.5036610047371626, 0.5185115777017428, 0.5321377915498797, 0.5441695314546555, 0.5550560513037092, 0.564888065360188, 0.5736895293487845, 0.581147374604847, 0.5875375874379745, 0.5929939887542812, 0.5975333391539005, 0.6009165175661537, 0.6032777584906068, 0.6047853561803952, 0.6054498253069145, 0.6050898980079614, 0.6037244457830772, 0.6015547245929572, 0.598586824358939, 0.5946886340708615, 0.5897786151068943, 0.5840933257932519, 0.5776384396307803, 0.5703321442305788, 0.5620831260629839, 0.5532569211134458, 0.5439421728921959, 0.534220103496476, 0.5242720970668235, 0.5143449252874155, 0.5043095867784212, 0.494008237327816, 0.4841180435773471, 0.4756155744654352, 0.46833942856748545, 0.4621337355785085, 0.4571591225284231, 0.45294190855697924, 0.44936770486777666, 0.44637403983907925, 0.44404552752341336, 0.442170745211681, 0.44065333379373495, 0.4394548655054464, 0.4385971613112518, 0.43797628772193486, 0.4375198216447284, 0.43720452335753984, 0.4370320317591272, 0.4369769936621983, 0.4370031379845354, 0.4371013087432195, 0.43727043943654054, 0.4374976506381246, 0.4377577087843371, 0.43804307882824595, 0.43834658277507565, 0.4386535547066741, 0.43895127504430503, 0.4392327676112338, 0.4394889084669748, 0.43969337746243636, 0.4398502241012521, 0.43995415741250216, 0.44};
  static const double p_T[] = {0.0, -0.0005221811762738234, -0.0016903984587716663, -0.0034386986305623736, -0.005701675290335035, -0.0085380760081868, -0.011695391654364902, -0.015102426723141285, -0.01871240526763499, -0.02250137805391348, -0.026357481697998657, -0.030229755843330656, -0.034082410056376924, -0.03783811945806834, -0.041422992817584804, -0.0448079553650357, -0.04794925912207289, -0.05070543947559343, -0.052945972695039915, -0.05467087662333323, -0.055803106937418745, -0.056105851863318394, -0.05523579627994845, -0.05324572797059415, -0.04996867304612585, -0.04500494437875532, -0.03750295192172132, -0.028771403208713565, -0.01974579391690079, -0.011693464849134935, -0.008096652838158762, -0.009608841923228162, -0.015581559668092364, -0.025147953577067863, -0.033887161306145726, -0.040326422292636936, -0.04474495335837997, -0.047433552081483664, -0.04864226170162168, -0.04891320265234658, -0.04838159296514437, -0.047119798925033665, -0.045014018506738605, -0.042324509370393025, -0.0391673433452299, -0.035584454747168454, -0.03152393465343954, -0.02709288844480279, -0.022375329129212706, -0.01738955699742373, -0.01209836905675343, -0.006521675619597833, -0.0007193547358525908, 0.005306450570424397, 0.011593902684640679, 0.018169320390633514, 0.02496524595390015, 0.031980118974453436, 0.039237383671598305, 0.0468379670040767, 0.054742825809817146, 0.06296365779665464, 0.07152655508536648, 0.08044707375638592, 0.08861282330271482, 0.09514607513241323, 0.09913026550481845, 0.10827762970700391, 0.11883044897732134, 0.13010955980611824, 0.14146426216041477, 0.15186405443091655, 0.16100687198920932, 0.1687808644364465, 0.17504127227096858, 0.17907548486391017, 0.1812561952716907, 0.18184979363440595, 0.18093489882819663, 0.17830781272825558, 0.1745190246957462, 0.16995358444512468, 0.1646996299813687, 0.15859639045380025, 0.1514420860546665, 0.14337600786181987, 0.13438026006653883, 0.12429317279456738, 0.11311144022975897, 0.1012148448690729, 0.0887149708002157, 0.07571196336372758, 0.06252783086556497, 0.04956290149156968, 0.037066257943782, 0.025371001155619598, 0.01557790086372325, 0.0077943311295240415, 0.0024544193404763153, -3.235259282696745e-16};

  iterationCount++;

  if (iterationCount >= 100) {
    iterationCount = 0;
  }

  float q1 = q1_T[iterationCount];
  float q2 = q2_T[iterationCount];
  float q3 = q3_T[iterationCount];
  float q4 = q4_T[iterationCount];

  float x = x_T[iterationCount];
  float y = y_T[iterationCount];
  float p = p_T[iterationCount];

  float roll = 0;
  float pitch = -p;
  float yaw = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  // new_state_est.body.pose.pose.orientation = last_imu_msg_->orientation;
  new_state_est.body.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
  new_state_est.body.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
  new_state_est.body.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
  new_state_est.body.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;

  // new_state_est.joints = *last_joint_state_msg_;

  new_state_est.joints.name.push_back("0");
  new_state_est.joints.name.push_back("1");
  new_state_est.joints.name.push_back("2");
  new_state_est.joints.name.push_back("3");
  new_state_est.joints.name.push_back("4");
  new_state_est.joints.name.push_back("5");
  new_state_est.joints.name.push_back("6");
  new_state_est.joints.name.push_back("7");
  new_state_est.joints.name.push_back("8");
  new_state_est.joints.name.push_back("9");
  new_state_est.joints.name.push_back("10");
  new_state_est.joints.name.push_back("11");

  new_state_est.joints.position.push_back(q3);
  new_state_est.joints.position.push_back(3.14159 - q4);
  new_state_est.joints.position.push_back(q1);
  new_state_est.joints.position.push_back(3.14159 - q2);
  new_state_est.joints.position.push_back(q3);
  new_state_est.joints.position.push_back(3.14159 - q4);
  new_state_est.joints.position.push_back(q1);
  new_state_est.joints.position.push_back(3.14159 - q2);
  new_state_est.joints.position.push_back(0);
  new_state_est.joints.position.push_back(0);
  new_state_est.joints.position.push_back(0);
  new_state_est.joints.position.push_back(0);
  
  // new_state_est.body.pose.pose.position = last_mocap_msg_->pose.position;
  new_state_est.body.pose.pose.position.x = x;
  new_state_est.body.pose.pose.position.y = 0;
  new_state_est.body.pose.pose.position.z = y;

  new_state_est.header.stamp = ros::Time::now();
  return new_state_est;
}

void TrajectoryVisualizationPublisher::spin() {
  ros::Rate r(25);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce(); 

    // Compute new state estimate
    spirit_msgs::StateEstimate new_state_est = this->updateStep();

    // Publish new state estimate
    ground_truth_state_pub_.publish(new_state_est);

    // Store new state estimate for next iteration
    last_state_est_ = new_state_est;

    // Enforce update rate
    r.sleep();
  }
}
