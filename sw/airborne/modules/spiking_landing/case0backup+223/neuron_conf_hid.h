//Auto-generated
#include "Neuron.h"
float const a_v_hid[] = {0.0, 1.951683521270752, 1.3781061172485352, 0.02987617254257202, 0.3354702591896057, 1.3051813840866089, 0.5274430513381958, 0.8465848565101624, 1.3186999559402466, 1.3748488426208496, 0.39659565687179565, 0.6170873045921326, 2.0, 0.06289935111999512, 2.0, 1.5524415969848633, 1.5026283264160156, 0.6018393635749817, 0.0, 0.0};
float const a_th_hid[] = {0.10215389728546143, 0.0, 0.9138231873512268, 0.3557276427745819, 1.8234853744506836, 2.0, 1.638917088508606, 1.1989201307296753, 2.0, 1.7166045904159546, 0.3236594796180725, 1.2857269048690796, 2.0, 2.0, 0.2904774844646454, 1.7233479022979736, 0.21136385202407837, 0.18679296970367432, 1.710437297821045, 1.4147372245788574};
float const a_t_hid[] = {0.5504514575004578, 0.550490140914917, 2.0, 1.679903507232666, 2.0, 1.605916976928711, 0.711180567741394, 2.0, 1.1210331916809082, 1.6839882135391235, 2.0, 1.709151029586792, 0.3148065209388733, 0.21169662475585938, 0.10495162010192871, 0.3239172101020813, 2.0, 0.3276134133338928, 1.064145803451538, 0.49404317140579224};
float const d_v_hid[] = {0.0, 0.8114686012268066, 0.8855603933334351, 0.4141518473625183, 0.17975342273712158, 1.0, 0.17887356877326965, 0.7196787595748901, 0.2828003168106079, 0.6465188264846802, 0.563660740852356, 0.8684435486793518, 0.0, 0.649483859539032, 0.0, 0.9153376221656799, 0.885161280632019, 1.0, 0.03984750807285309, 0.5837230086326599};
float const d_th_hid[] = {0.21176230907440186, 0.1557551920413971, 1.0, 1.0, 0.1947704553604126, 0.5504199862480164, 0.6540303230285645, 0.25495535135269165, 0.5449889302253723, 0.6463640928268433, 0.29179948568344116, 0.835401177406311, 0.41355371475219727, 0.0, 1.0, 0.7691766023635864, 0.17507991194725037, 0.0, 1.0, 0.28556129336357117};
float const d_t_hid[] = {0.29999297857284546, 0.6425609588623047, 0.20404431223869324, 0.0, 0.6454000473022461, 0.3221021294593811, 0.6612693667411804, 1.0, 0.0, 0.12186864018440247, 0.8374072313308716, 1.0, 0.20340025424957275, 0.3095553517341614, 0.7986657023429871, 0.5205466747283936, 0.1590425819158554, 1.0, 0.023057028651237488, 0.43953219056129456};
float const th_rest_hid[] = {0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224, 0.20000000298023224};
NeuronConf const conf_hid = {1, 20, a_v_hid, a_th_hid, a_t_hid, d_v_hid, d_th_hid, d_t_hid, 0.0, th_rest_hid};
