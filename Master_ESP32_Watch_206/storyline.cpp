/*
 * storyline.cpp - Story Mode with NVS + Full Dialogue + Boss Sprites
 * FUSION OS - Complete Story Implementation
 */

#include "storyline.h"
#include "config.h"
#include "themes.h"
#include "xp_system.h"
#include "gacha.h"
#include "display.h"
#include "touch.h"
#include "navigation.h"  

StorySystemState story_system;
CharacterStory stories[THEME_COUNT];

static const int CHAPTER_LEVELS[MAX_CHAPTERS_PER_CHARACTER] = {
 CHAPTER_1_LEVEL, CHAPTER_2_LEVEL, CHAPTER_3_LEVEL, CHAPTER_4_LEVEL, CHAPTER_5_LEVEL,
 CHAPTER_6_LEVEL, CHAPTER_7_LEVEL, CHAPTER_8_LEVEL, CHAPTER_9_LEVEL, CHAPTER_10_LEVEL,
 CHAPTER_11_LEVEL, CHAPTER_12_LEVEL, CHAPTER_13_LEVEL, CHAPTER_14_LEVEL, CHAPTER_15_LEVEL
};

static const char* STORY_NAMES[THEME_COUNT] = {
 "Road to Pirate King", "Rise of the Shadow Monarch", "Quest for the Eliacube",
 "Legend of the Seventh", "Path to Ultra Instinct", "Sun Breathing Legacy",
 "The Strongest Sorcerer", "Wings of Freedom", "Hero for Fun",
 "Symbol of Peace", "Elemental Hero Rising"
};

static const char* CHAPTER_TITLES[THEME_COUNT][MAX_CHAPTERS_PER_CHARACTER] = {
 // Luffy - 15 chapters
 {"Romance Dawn", "Orange Town Circus", "Syrup Village", "Baratie Showdown", "Arlong Park", 
  "Alabasta Rising", "Enies Lobby", "Thriller Bark", "Sabaody Separation", "Marineford War",
  "Two Years Training", "Fishman Island", "Dressrosa Liberation", "Whole Cake Escape", "Wano: Gear 5 Dawn"},
 // Jinwoo - 15 chapters
 {"Double Dungeon", "Instance Dungeon", "Penalty Quest", "Red Gate", "Demon Castle", 
  "Return to Jeju", "Architect's Design", "National Level", "Monarchs War", "Shadow Army Rising",
  "S-Rank Hunter", "Japan Crisis", "Rulers Alliance", "Dragon Emperor", "Final Battle: Antares"},
 // Yugo - 15 chapters
 {"The Boy with the Hat", "Nox's Shadow", "Brotherhood of the Tofu", "The Dragon's Call", "Nox Awakens", 
  "Qilby's Betrayal", "Realm of Shushu", "Count's Ice", "Ogrest's Tears", "Oropo's Mirror",
  "Eliatrope Council", "Zinit Awakening", "Dimension Walker", "Brotherhood Reunited", "Master of Portals"},
 // Naruto - 15 chapters
 {"Leaf Village Outcast", "Land of Waves", "Chunin Exams", "Sasuke Retrieval", "Sage Training", 
  "Pain's Assault", "Fourth Great War", "Madara Rises", "Kaguya Awakens", "The Last Battle",
  "Boruto Generation", "Otsutsuki Threat", "Kurama's End", "Baryon Mode", "Village Protector"},
 // Goku - 15 chapters
 {"Arrival of Raditz", "Namek Saga", "Frieza's Wrath", "Cell Games", "Majin Buu Arc", 
  "Battle of Gods", "Universe 6 Tournament", "Goku Black Saga", "Tournament of Power", "Broly Encounter",
  "Moro Arc", "Granolah Saga", "Ultra Instinct Perfected", "Beast Gohan Rising", "Ultra Instinct Mastered"},
 // Tanjiro - 15 chapters
 {"Final Selection", "First Mission", "Asakusa Arc", "Mt. Natagumo", "Mugen Train", 
  "Entertainment District", "Swordsmith Village", "Hashira Training", "Infinity Castle", "Sunrise Countdown",
  "Upper Moon Assault", "Demon King Confrontation", "Sun Breathing Mastery", "Nezuko's Humanity", "Dawn of Victory"},
 // Gojo - 15 chapters
 {"Cursed Womb", "Kyoto Goodwill", "Vs. Mahito", "Shibuya Incident", "Hidden Inventory", 
  "Perfect Preparation", "Culling Game", "Colony Assault", "Bath Ritual", "Shinjuku Showdown",
  "Prison Realm Escape", "Six Eyes Awakening", "Unlimited Void Perfected", "Sukuna Confrontation", "Strongest Sorcerer"},
 // Levi - 15 chapters
 {"104th Cadet Corps", "Trost District", "Female Titan", "Clash of Titans", "Uprising Arc", 
  "Return to Shiganshina", "Marley Arc", "War for Paradis", "Rumbling", "Final Stand",
  "Kenny's Legacy", "Ackerman Bloodline", "Beyond the Walls", "Freedom's Cost", "Humanity's Last Hope"},
 // Saitama - 15 chapters
 {"Bald Cape", "House of Evolution", "Sea King Crisis", "Boros Ship", "Garou Intro", 
  "Monster Association", "Hero Hunt", "Monster Garou", "Cosmic Fear", "Limiter Theory",
  "Hero Name Fame", "S-Class Recognition", "Monster King", "Serious Series", "One Punch Legend"},
 // Deku - 15 chapters
 {"Entrance Exam", "USJ Incident", "Sports Festival", "Hero Killer", "Training Camp", 
  "Overhaul Raid", "Joint Training", "Paranormal Liberation", "Dark Hero Arc", "Final War",
  "One For All Mastery", "Shigaraki Confrontation", "All Might's Legacy", "Quirk Singularity", "Symbol of Peace"},
 // BoBoiBoy - 15 chapters
 {"Power Sphera Awakening", "Adu Du's Attack", "BoBoiBoy Split", "Ejo Jo Invasion", "Galaxy Mission Start", 
  "Retak'ka Returns", "Power Sphera Hunt", "Elemental Fusion", "Supra Awakening", "Final Elemental Battle",
  "TAPOPS Academy", "Captain Kaizo's Trial", "Seven Elements Mastery", "Kuasa Tujuh", "Ultimate Elemental Hero"}
};

// Yugo alt path titles - 15 chapters
static const char* YUGO_DRAGON_TITLES[MAX_CHAPTERS_PER_CHARACTER] = {
 "Birth of the Dragon", "Dragon's Call", "Wings of Destiny",
 "The Dragon King's Legacy", "Flames of Adamai", "Council of Six",
 "Shinonome Rising", "Eliatrope Dragon Bond", "The Dragon God", "Ruler of Dragons",
 "Dragon Brotherhood", "Primordial Dofus", "Ancient Dragon War", "Dragon King's Return", "Master of Dragons"
};
static const char* YUGO_PORTAL_TITLES[MAX_CHAPTERS_PER_CHARACTER] = {
 "Portal Awakening", "Gateway Training", "Path of Portals",
 "Dimensional Mastery", "Wakfu Convergence", "The Eliacube's Secret",
 "Realm Walker", "Time and Space", "The Krosmoz Revealed", "Master of Dimensions",
 "Portal Network", "Dimensional Rifts", "Multiverse Travel", "Portal King Rising", "Krosmoz Guardian"
};

// =============================================================================
// HELPER: Set a dialogue entry
// =============================================================================
static void setDlg(DialogueScreen* d, const char* spk, const char* txt, uint16_t col,
 bool choice = false, const char* c1 = "", const char* c2 = "",
 int n1 = 0, int n2 = 0) {
 d->speaker = spk; d->text = txt; d->speaker_color = col;
 d->has_choices = choice; d->choice1 = c1; d->choice2 = c2;
 d->choice1_next_index = n1; d->choice2_next_index = n2;
}

// =============================================================================
// BOSS DATA (with visual type + colors for sprites)
// =============================================================================
static const StoryBoss ALL_BOSSES[THEME_COUNT][MAX_CHAPTERS_PER_CHARACTER] = {
 { // Luffy
   {"Alvida", "One Piece", 100,10,5,50,25,-1, BOSS_HUMANOID, RGB565(180,80,120), RGB565(255,200,200)},
   {"Buggy", "One Piece", 150,15,8,75,35,-1, BOSS_HUMANOID, RGB565(200,50,50), RGB565(100,100,255)},
   {"Kuro", "One Piece", 200,18,10,100,50,1, BOSS_HUMANOID, RGB565(30,30,30), RGB565(200,200,200)},
   {"Don Krieg", "One Piece", 250,22,12,125,60,-1, BOSS_GIANT, RGB565(180,160,50), RGB565(100,100,100)},
   {"Arlong", "One Piece", 350,28,15,150,75,2, BOSS_BEAST, RGB565(60,80,180), RGB565(200,200,220)},
   {"Crocodile", "One Piece", 500,35,20,200,100,3, BOSS_ELEMENTAL, RGB565(180,160,100), RGB565(255,220,150)},
   {"Rob Lucci", "One Piece", 650,42,25,250,125,-1, BOSS_BEAST, RGB565(60,60,80), RGB565(255,255,255)},
   {"Moria", "One Piece", 720,45,27,260,130,-1, BOSS_DEMON, RGB565(80,60,120), RGB565(200,180,220)},
   {"Magellan", "One Piece", 800,50,30,300,150,4, BOSS_DEMON, RGB565(120,40,120), RGB565(200,100,200)},
   {"Akainu", "One Piece", 1000,60,35,400,200,5, BOSS_ELEMENTAL, RGB565(200,50,30), RGB565(255,150,50)},
   {"Hody Jones", "One Piece", 1100,65,38,420,210,-1, BOSS_BEAST, RGB565(60,100,180), RGB565(150,200,255)},
   {"Doflamingo", "One Piece", 1300,75,42,480,240,6, BOSS_HUMANOID, RGB565(255,100,150), RGB565(255,200,200)},
   {"Katakuri", "One Piece", 1500,85,48,550,280,7, BOSS_HUMANOID, RGB565(120,30,60), RGB565(200,150,180)},
   {"Big Mom", "One Piece", 1800,100,55,620,320,-1, BOSS_GIANT, RGB565(255,150,200), RGB565(255,220,220)},
   {"Kaido", "One Piece", 2200,120,65,750,400,8, BOSS_BEAST, RGB565(100,60,180), RGB565(255,200,50)}
 },
 { // Jinwoo
   {"Low Rank Orc", "Solo Leveling", 100,10,5,50,25,-1, BOSS_BEAST, RGB565(80,120,50), RGB565(40,60,20)},
   {"Stone Golem", "Solo Leveling", 150,15,10,75,35,-1, BOSS_GIANT, RGB565(120,120,120), RGB565(80,80,80)},
   {"Ice Elf", "Solo Leveling", 200,20,12,100,50,10, BOSS_ELEMENTAL, RGB565(150,200,255), RGB565(200,230,255)},
   {"Cerberus", "Solo Leveling", 300,25,15,150,75,-1, BOSS_BEAST, RGB565(80,20,20), RGB565(255,80,30)},
   {"Igris", "Solo Leveling", 400,35,20,200,100,11, BOSS_HUMANOID, RGB565(200,50,50), RGB565(255,200,50)},
   {"Tusk", "Solo Leveling", 500,40,22,250,125,-1, BOSS_GIANT, RGB565(60,100,60), RGB565(200,200,200)},
   {"Baran", "Solo Leveling", 700,50,30,300,150,12, BOSS_DEMON, RGB565(180,50,50), RGB565(255,150,50)},
   {"Architect", "Solo Leveling", 900,60,35,400,200,-1, BOSS_HUMANOID, RGB565(200,200,220), RGB565(150,150,200)},
   {"Beast Monarch", "Solo Leveling", 1200,75,45,450,250,13, BOSS_BEAST, RGB565(150,80,30), RGB565(255,200,100)},
   {"Frost Monarch", "Solo Leveling", 1400,85,52,520,280,-1, BOSS_ELEMENTAL, RGB565(100,180,255), RGB565(200,230,255)},
   {"Beru", "Solo Leveling", 1600,95,58,580,320,14, BOSS_BEAST, RGB565(60,120,60), RGB565(150,255,150)},
   {"Iron Body Monarch", "Solo Leveling", 1800,105,62,640,360,-1, BOSS_GIANT, RGB565(150,150,160), RGB565(200,200,200)},
   {"Plague Monarch", "Solo Leveling", 2000,115,68,700,400,-1, BOSS_DEMON, RGB565(100,80,60), RGB565(200,150,100)},
   {"Dragon Emperor", "Solo Leveling", 2300,130,75,780,450,15, BOSS_BEAST, RGB565(200,150,50), RGB565(255,220,100)},
   {"Antares", "Solo Leveling", 2800,160,90,900,550,16, BOSS_DEMON, RGB565(200,30,30), RGB565(255,100,100)}
 },
 { // Yugo
   {"Rubilax", "Wakfu", 100,10,5,50,25,-1, BOSS_DEMON, RGB565(180,80,30), RGB565(255,150,50)},
   {"Vampyro", "Wakfu", 150,15,8,75,35,-1, BOSS_HUMANOID, RGB565(80,30,80), RGB565(200,100,200)},
   {"Nox's Puppet", "Wakfu", 200,18,10,100,50,20, BOSS_HUMANOID, RGB565(180,160,80), RGB565(100,80,40)},
   {"Razortime", "Wakfu", 280,24,14,140,70,-1, BOSS_HUMANOID, RGB565(160,140,60), RGB565(200,180,80)},
   {"Nox", "Wakfu", 400,35,20,200,100,21, BOSS_HUMANOID, RGB565(180,160,80), RGB565(60,200,200)},
   {"Qilby", "Wakfu", 550,42,25,260,130,22, BOSS_HUMANOID, RGB565(200,200,220), RGB565(100,200,255)},
   {"Rushu", "Wakfu", 700,50,30,320,160,-1, BOSS_DEMON, RGB565(200,30,30), RGB565(255,100,30)},
   {"Count Harebourg", "Wakfu", 850,58,35,380,190,23, BOSS_HUMANOID, RGB565(100,150,220), RGB565(200,220,255)},
   {"Ogrest", "Wakfu", 1100,70,45,460,240,24, BOSS_GIANT, RGB565(60,150,60), RGB565(100,200,100)},
   {"Oropo", "Wakfu", 1400,85,52,540,300,25, BOSS_HUMANOID, RGB565(0,180,220), RGB565(255,200,80)},
   {"Lady Echo", "Wakfu", 1600,92,58,600,340,-1, BOSS_HUMANOID, RGB565(200,150,200), RGB565(255,200,255)},
   {"Mechasm Commander", "Wakfu", 1800,100,62,660,380,-1, BOSS_GIANT, RGB565(150,150,180), RGB565(200,200,220)},
   {"Krosmoz Guardian", "Wakfu", 2000,110,68,720,420,26, BOSS_ELEMENTAL, RGB565(100,200,255), RGB565(200,255,255)},
   {"Adamai Corrupted", "Wakfu", 2300,125,75,800,480,-1, BOSS_BEAST, RGB565(80,60,120), RGB565(200,150,255)},
   {"The Great Dragon", "Wakfu", 2700,150,85,900,550,27, BOSS_DEMON, RGB565(200,180,50), RGB565(255,255,200)}
 },
 { // Naruto
   {"Mizuki", "Naruto", 100,10,5,50,25,-1, BOSS_HUMANOID, RGB565(150,150,160), RGB565(200,200,210)},
   {"Zabuza", "Naruto", 180,18,10,90,45,-1, BOSS_HUMANOID, RGB565(80,80,100), RGB565(200,200,220)},
   {"Orochimaru", "Naruto", 250,24,14,130,65,30, BOSS_DEMON, RGB565(140,120,180), RGB565(200,200,100)},
   {"Gaara", "Naruto", 320,30,18,170,85,-1, BOSS_ELEMENTAL, RGB565(180,150,100), RGB565(200,50,50)},
   {"Itachi", "Naruto", 420,38,22,220,110,31, BOSS_HUMANOID, RGB565(200,30,30), RGB565(30,30,30)},
   {"Pain", "Naruto", 600,50,30,300,150,32, BOSS_HUMANOID, RGB565(200,100,30), RGB565(150,100,180)},
   {"Obito", "Naruto", 800,62,38,380,190,-1, BOSS_HUMANOID, RGB565(200,100,30), RGB565(30,30,30)},
   {"Madara", "Naruto", 1000,75,45,450,225,33, BOSS_HUMANOID, RGB565(180,30,30), RGB565(40,40,60)},
   {"Kaguya", "Naruto", 1300,90,55,520,280,34, BOSS_DEMON, RGB565(230,230,240), RGB565(200,100,200)},
   {"Isshiki", "Naruto", 1500,105,65,600,340,35, BOSS_HUMANOID, RGB565(80,60,80), RGB565(200,200,200)},
   {"Momoshiki", "Naruto", 1700,115,70,660,380,-1, BOSS_DEMON, RGB565(180,150,200), RGB565(255,200,255)},
   {"Code", "Naruto", 1900,125,75,720,420,-1, BOSS_HUMANOID, RGB565(100,100,120), RGB565(200,200,220)},
   {"Kurama Final", "Naruto", 2100,135,80,780,460,36, BOSS_BEAST, RGB565(200,100,30), RGB565(255,200,50)},
   {"Baryon Threat", "Naruto", 2400,150,88,850,500,-1, BOSS_ELEMENTAL, RGB565(255,180,50), RGB565(255,255,200)},
   {"Otsutsuki God", "Naruto", 2800,175,100,950,580,37, BOSS_DEMON, RGB565(220,220,240), RGB565(200,150,255)}
 },
 { // Goku
   {"Raditz", "Dragon Ball", 120,12,6,60,30,-1, BOSS_HUMANOID, RGB565(60,40,30), RGB565(200,200,200)},
   {"Nappa", "Dragon Ball", 180,18,10,90,45,-1, BOSS_GIANT, RGB565(100,80,60), RGB565(180,160,100)},
   {"Vegeta", "Dragon Ball", 280,28,16,140,70,40, BOSS_HUMANOID, RGB565(40,40,120), RGB565(255,200,50)},
   {"Frieza", "Dragon Ball", 380,38,22,200,100,-1, BOSS_HUMANOID, RGB565(200,150,220), RGB565(200,50,80)},
   {"Frieza Final", "Dragon Ball", 500,50,30,280,140,41, BOSS_HUMANOID, RGB565(220,220,230), RGB565(200,50,200)},
   {"Cell", "Dragon Ball", 700,65,38,360,180,42, BOSS_HUMANOID, RGB565(80,140,80), RGB565(30,60,30)},
   {"Majin Buu", "Dragon Ball", 900,80,48,440,220,-1, BOSS_DEMON, RGB565(240,180,200), RGB565(200,100,120)},
   {"Goku Black", "Dragon Ball", 1100,92,55,500,260,-1, BOSS_HUMANOID, RGB565(30,30,30), RGB565(200,50,200)},
   {"Beerus", "Dragon Ball", 1300,105,62,560,300,43, BOSS_HUMANOID, RGB565(140,100,180), RGB565(200,180,220)},
   {"Jiren", "Dragon Ball", 1600,120,72,640,360,44, BOSS_GIANT, RGB565(180,30,30), RGB565(200,200,200)},
   {"Moro", "Dragon Ball", 1800,132,78,700,400,45, BOSS_DEMON, RGB565(60,60,80), RGB565(100,200,100)},
   {"Granolah", "Dragon Ball", 2000,142,84,760,440,-1, BOSS_HUMANOID, RGB565(100,120,80), RGB565(200,220,150)},
   {"Gas", "Dragon Ball", 2200,155,90,820,480,-1, BOSS_GIANT, RGB565(150,80,50), RGB565(255,200,100)},
   {"Beast Gohan", "Dragon Ball", 2500,170,98,900,530,46, BOSS_HUMANOID, RGB565(200,200,220), RGB565(255,100,100)},
   {"Ultra Jiren", "Dragon Ball", 2900,195,110,1000,600,47, BOSS_GIANT, RGB565(220,30,30), RGB565(255,255,255)}
 },
 { // Tanjiro
   {"Hand Demon", "Demon Slayer", 100,10,5,50,25,-1, BOSS_DEMON, RGB565(80,60,60), RGB565(200,100,100)},
   {"Swamp Demon", "Demon Slayer", 160,16,9,80,40,-1, BOSS_DEMON, RGB565(60,80,60), RGB565(100,200,100)},
   {"Susamaru", "Demon Slayer", 220,22,13,120,60,50, BOSS_HUMANOID, RGB565(200,100,50), RGB565(255,200,100)},
   {"Kyogai", "Demon Slayer", 300,30,18,160,80,-1, BOSS_DEMON, RGB565(100,80,60), RGB565(200,180,100)},
   {"Rui", "Demon Slayer", 420,40,24,220,110,51, BOSS_HUMANOID, RGB565(220,220,230), RGB565(200,200,255)},
   {"Enmu", "Demon Slayer", 550,52,32,290,145,52, BOSS_DEMON, RGB565(60,40,100), RGB565(150,100,200)},
   {"Daki & Gyutaro", "Demon Slayer", 720,68,42,370,185,-1, BOSS_HUMANOID, RGB565(200,100,150), RGB565(80,120,80)},
   {"Hantengu", "Demon Slayer", 900,85,52,440,220,53, BOSS_DEMON, RGB565(120,100,80), RGB565(200,180,100)},
   {"Akaza", "Demon Slayer", 1150,105,65,520,280,54, BOSS_HUMANOID, RGB565(200,80,120), RGB565(100,150,200)},
   {"Kokushibo", "Demon Slayer", 1400,120,72,600,340,55, BOSS_HUMANOID, RGB565(80,40,80), RGB565(200,100,200)},
   {"Douma", "Demon Slayer", 1600,132,78,660,380,-1, BOSS_HUMANOID, RGB565(200,200,220), RGB565(150,200,255)},
   {"Nakime", "Demon Slayer", 1800,140,82,720,420,-1, BOSS_DEMON, RGB565(100,80,100), RGB565(200,180,200)},
   {"Muzan Form 2", "Demon Slayer", 2100,155,90,800,480,56, BOSS_DEMON, RGB565(60,0,0), RGB565(200,30,30)},
   {"Nezuko Awakened", "Demon Slayer", 2400,170,98,880,540,-1, BOSS_HUMANOID, RGB565(255,150,200), RGB565(200,100,150)},
   {"Muzan Final", "Demon Slayer", 2800,195,110,1000,620,57, BOSS_DEMON, RGB565(40,0,0), RGB565(255,50,50)}
 },
 { // Gojo
   {"Cursed Spirit E", "JJK", 100,10,5,50,25,-1, BOSS_DEMON, RGB565(60,80,60), RGB565(100,150,100)},
   {"Cursed Spirit D", "JJK", 150,15,8,75,38,-1, BOSS_DEMON, RGB565(80,60,80), RGB565(150,100,150)},
   {"Finger Bearer", "JJK", 220,22,13,120,60,60, BOSS_DEMON, RGB565(140,60,60), RGB565(200,100,100)},
   {"Hanami", "JJK", 320,32,20,180,90,-1, BOSS_ELEMENTAL, RGB565(60,120,60), RGB565(200,100,50)},
   {"Jogo", "JJK", 450,45,28,250,125,61, BOSS_ELEMENTAL, RGB565(200,80,30), RGB565(255,200,50)},
   {"Dagon", "JJK", 580,55,35,320,160,-1, BOSS_ELEMENTAL, RGB565(40,80,150), RGB565(100,150,200)},
   {"Mahito", "JJK", 750,70,45,400,200,62, BOSS_HUMANOID, RGB565(100,120,140), RGB565(200,200,220)},
   {"Choso", "JJK", 900,85,52,470,235,-1, BOSS_HUMANOID, RGB565(140,60,60), RGB565(200,100,100)},
   {"Toji", "JJK", 1150,110,68,550,300,63, BOSS_HUMANOID, RGB565(40,40,50), RGB565(200,200,200)},
   {"Kenjaku", "JJK", 1400,125,75,620,350,-1, BOSS_HUMANOID, RGB565(120,100,80), RGB565(200,180,150)},
   {"Yorozu", "JJK", 1600,135,80,680,390,64, BOSS_HUMANOID, RGB565(180,150,200), RGB565(255,200,255)},
   {"Kashimo", "JJK", 1800,145,85,740,430,-1, BOSS_ELEMENTAL, RGB565(200,200,100), RGB565(255,255,200)},
   {"Mahoraga", "JJK", 2100,160,92,820,480,-1, BOSS_DEMON, RGB565(30,60,30), RGB565(100,200,100)},
   {"Sukuna 2 Fingers", "JJK", 2400,180,100,900,540,65, BOSS_DEMON, RGB565(180,60,80), RGB565(255,200,200)},
   {"Sukuna Full", "JJK", 2900,210,115,1050,650,66, BOSS_DEMON, RGB565(200,30,30), RGB565(255,255,255)}
 },
 { // Levi
   {"Titan 3m", "AoT", 80,8,4,45,22,-1, BOSS_GIANT, RGB565(180,140,100), RGB565(200,100,80)},
   {"Titan 7m", "AoT", 140,14,8,75,38,-1, BOSS_GIANT, RGB565(160,120,90), RGB565(200,80,60)},
   {"Abnormal", "AoT", 200,20,12,110,55,70, BOSS_GIANT, RGB565(180,150,110), RGB565(200,60,60)},
   {"Female Titan", "AoT", 320,32,20,180,90,-1, BOSS_GIANT, RGB565(200,180,160), RGB565(100,150,200)},
   {"Armored Titan", "AoT", 500,45,35,280,140,71, BOSS_GIANT, RGB565(180,160,120), RGB565(200,200,200)},
   {"Colossal", "AoT", 650,55,40,350,175,72, BOSS_GIANT, RGB565(200,80,60), RGB565(200,200,200)},
   {"Beast Titan", "AoT", 800,70,48,420,210,-1, BOSS_BEAST, RGB565(140,120,80), RGB565(100,80,50)},
   {"War Hammer", "AoT", 950,85,55,490,250,73, BOSS_GIANT, RGB565(220,220,230), RGB565(200,200,200)},
   {"Attack Titan", "AoT", 1150,100,65,560,300,74, BOSS_GIANT, RGB565(100,140,80), RGB565(200,200,200)},
   {"Founding", "AoT", 1400,120,75,640,360,75, BOSS_GIANT, RGB565(220,200,180), RGB565(255,255,255)},
   {"Kenny Ackerman", "AoT", 1600,130,80,700,400,-1, BOSS_HUMANOID, RGB565(80,60,50), RGB565(200,200,200)},
   {"Jaw Titan", "AoT", 1800,140,85,760,440,-1, BOSS_BEAST, RGB565(180,150,100), RGB565(200,200,200)},
   {"Cart Titan", "AoT", 2000,150,90,820,480,76, BOSS_BEAST, RGB565(150,130,100), RGB565(200,200,200)},
   {"Eren Titan Form", "AoT", 2300,170,100,900,540,-1, BOSS_GIANT, RGB565(100,140,80), RGB565(200,200,200)},
   {"Founding Titan", "AoT", 2700,200,115,1050,650,77, BOSS_GIANT, RGB565(200,180,150), RGB565(255,255,255)}
 },
 { // Saitama
   {"Crablante", "OPM", 50,5,2,30,15,-1, BOSS_BEAST, RGB565(180,60,40), RGB565(200,100,80)},
   {"Mosquito Girl", "OPM", 100,10,5,60,30,-1, BOSS_HUMANOID, RGB565(150,80,60), RGB565(200,150,100)},
   {"Deep Sea King", "OPM", 200,20,12,120,60,80, BOSS_BEAST, RGB565(60,100,150), RGB565(200,200,220)},
   {"Genos Spar", "OPM", 280,28,18,160,80,-1, BOSS_HUMANOID, RGB565(255,200,50), RGB565(30,30,30)},
   {"Boros G1", "OPM", 400,40,25,230,115,81, BOSS_HUMANOID, RGB565(100,60,150), RGB565(200,150,255)},
   {"Boros G2", "OPM", 600,55,35,320,160,82, BOSS_HUMANOID, RGB565(200,150,255), RGB565(255,200,255)},
   {"Garou Human", "OPM", 750,70,45,400,200,-1, BOSS_HUMANOID, RGB565(200,200,220), RGB565(200,30,30)},
   {"Garou Monster", "OPM", 1000,95,60,500,280,83, BOSS_BEAST, RGB565(30,30,40), RGB565(200,50,50)},
   {"Cosmic Garou", "OPM", 1400,130,80,620,380,84, BOSS_DEMON, RGB565(30,30,60), RGB565(255,255,255)},
   {"God", "OPM", 1700,150,90,720,440,85, BOSS_DEMON, RGB565(20,20,30), RGB565(255,255,220)},
   {"Psykos Fusion", "OPM", 1900,160,95,780,480,-1, BOSS_DEMON, RGB565(80,60,120), RGB565(200,150,255)},
   {"Platinum Sperm", "OPM", 2100,170,100,840,520,-1, BOSS_HUMANOID, RGB565(200,200,220), RGB565(255,255,255)},
   {"Orochi", "OPM", 2300,180,105,900,560,86, BOSS_BEAST, RGB565(100,60,30), RGB565(200,150,50)},
   {"Serious Saitama", "OPM", 2600,200,115,980,600,-1, BOSS_HUMANOID, RGB565(255,220,50), RGB565(200,30,30)},
   {"Void God", "OPM", 3000,240,130,1100,700,87, BOSS_DEMON, RGB565(10,10,20), RGB565(255,255,255)}
 },
 { // Deku
   {"Sludge Villain", "MHA", 80,8,4,45,22,-1, BOSS_ELEMENTAL, RGB565(80,100,60), RGB565(100,150,80)},
   {"Robot 0-P", "MHA", 150,15,10,85,42,-1, BOSS_GIANT, RGB565(100,100,110), RGB565(200,50,50)},
   {"Nomu USJ", "MHA", 250,25,18,140,70,90, BOSS_BEAST, RGB565(40,40,50), RGB565(200,200,200)},
   {"Stain", "MHA", 350,35,22,200,100,-1, BOSS_HUMANOID, RGB565(150,30,30), RGB565(200,200,200)},
   {"Muscular", "MHA", 480,48,30,270,135,91, BOSS_HUMANOID, RGB565(180,100,80), RGB565(200,60,60)},
   {"Overhaul", "MHA", 620,60,40,350,175,92, BOSS_HUMANOID, RGB565(80,40,60), RGB565(200,180,100)},
   {"High-End Nomu", "MHA", 800,78,50,440,220,-1, BOSS_BEAST, RGB565(60,60,80), RGB565(200,200,200)},
   {"Re-Destro", "MHA", 950,92,58,520,260,93, BOSS_HUMANOID, RGB565(120,60,60), RGB565(200,200,200)},
   {"Shigaraki", "MHA", 1200,115,72,620,340,94, BOSS_HUMANOID, RGB565(150,150,160), RGB565(200,30,30)},
   {"All For One", "MHA", 1500,140,85,720,420,95, BOSS_DEMON, RGB565(30,30,40), RGB565(200,30,30)},
   {"Gigantomachia", "MHA", 1700,150,90,780,460,-1, BOSS_GIANT, RGB565(120,100,80), RGB565(200,200,200)},
   {"Dabi Final", "MHA", 1900,160,95,840,500,-1, BOSS_ELEMENTAL, RGB565(100,150,200), RGB565(200,50,50)},
   {"Toga Twice", "MHA", 2100,170,100,900,540,96, BOSS_HUMANOID, RGB565(200,150,100), RGB565(255,200,200)},
   {"Shigaraki Evolved", "MHA", 2400,190,110,980,600,-1, BOSS_DEMON, RGB565(150,150,160), RGB565(200,30,30)},
   {"All For One Final", "MHA", 2800,220,125,1100,700,97, BOSS_DEMON, RGB565(20,20,30), RGB565(255,50,50)}
 },
 { // BoBoiBoy
   {"Adu Du Robot", "BBB", 100,10,5,50,25,-1, BOSS_HUMANOID, RGB565(80,150,80), RGB565(200,200,200)},
   {"Probe Mech", "BBB", 160,16,9,80,40,-1, BOSS_GIANT, RGB565(200,100,200), RGB565(255,200,255)},
   {"Ejo Jo", "BBB", 240,24,15,130,65,100, BOSS_HUMANOID, RGB565(200,50,50), RGB565(200,200,200)},
   {"Captain Kaizo", "BBB", 350,35,22,190,95,-1, BOSS_HUMANOID, RGB565(100,50,150), RGB565(200,150,255)},
   {"Retak'ka", "BBB", 500,50,32,280,140,101, BOSS_ELEMENTAL, RGB565(200,100,30), RGB565(255,200,80)},
   {"Bora Ra", "BBB", 650,62,40,360,180,102, BOSS_HUMANOID, RGB565(200,30,30), RGB565(200,200,200)},
   {"Reramos", "BBB", 820,78,50,450,225,-1, BOSS_GIANT, RGB565(80,80,100), RGB565(200,200,200)},
   {"Dark Klamkabot", "BBB", 1000,95,60,540,280,103, BOSS_DEMON, RGB565(40,30,60), RGB565(200,100,255)},
   {"Cristalbot", "BBB", 1250,115,75,640,350,104, BOSS_ELEMENTAL, RGB565(150,200,255), RGB565(255,255,255)},
   {"Dark Fang", "BBB", 1500,135,85,740,420,105, BOSS_DEMON, RGB565(30,0,30), RGB565(200,50,200)},
   {"TAPOPS Commander", "BBB", 1700,145,90,800,460,-1, BOSS_HUMANOID, RGB565(100,120,200), RGB565(200,220,255)},
   {"Captain Kaizo V2", "BBB", 1900,155,95,860,500,-1, BOSS_HUMANOID, RGB565(120,60,180), RGB565(220,180,255)},
   {"Retak'ka Evolved", "BBB", 2100,170,102,930,550,106, BOSS_ELEMENTAL, RGB565(220,120,40), RGB565(255,220,100)},
   {"Elemental Titan", "BBB", 2400,190,110,1000,600,-1, BOSS_GIANT, RGB565(200,180,50), RGB565(255,255,200)},
   {"Dark BoBoiBoy", "BBB", 2800,220,125,1150,700,107, BOSS_DEMON, RGB565(30,0,50), RGB565(255,100,255)}
 }
};

// =============================================================================
// NVS KEY HELPERS
// =============================================================================
static void getCharKey(ThemeType t, const char* s, char* b, size_t n) { snprintf(b,n,"s%d_%s",(int)t,s); }
static void getChKey(ThemeType t, int c, const char* s, char* b, size_t n) { snprintf(b,n,"s%d_c%d_%s",(int)t,c,s); }

// =============================================================================
// DIALOGUE INITIALIZATION
// =============================================================================
void initStoryDialogues() {
 // --- LUFFY ---
 CharacterStory* s = &stories[THEME_LUFFY_GEAR5];
 setDlg(&s->chapters[0].dialogues[0], "Luffy", "I'm gonna be King of the Pirates!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[0].dialogues[1], "Shanks", "This straw hat... I'll trust it to you, Luffy.", LUFFY_STRAW_RED);
 setDlg(&s->chapters[0].dialogues[2], "Narrator", "Years later, a rubber boy sets sail alone...", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[3], "Coby", "P-pirates are scary! You can't just--", COLOR_PINK);
 setDlg(&s->chapters[0].dialogues[4], "Luffy", "Shishishi! A pirate's life is all about freedom!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[0].dialogues[5], "Narrator", "But first... Alvida blocks the way!", COLOR_WHITE);
 s->chapters[0].dialogue_count = 6;

 setDlg(&s->chapters[1].dialogues[0], "Nami", "That clown pirate has terrorized this town...", NARUTO_RASENGAN_BLUE);
 setDlg(&s->chapters[1].dialogues[1], "Buggy", "MY NOSE ISN'T BIG! Flashy punishment time!", COLOR_RED);
 setDlg(&s->chapters[1].dialogues[2], "Luffy", "Your nose IS big. And your jokes are worse!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[1].dialogues[3], "Zoro", "Captain... this is what I signed up for?", LEVI_SURVEY_GREEN);
 setDlg(&s->chapters[1].dialogues[4], "Luffy", "Gomu Gomu no... PISTOL!", LUFFY_SUN_GOLD);
 s->chapters[1].dialogue_count = 5;

 setDlg(&s->chapters[2].dialogues[0], "Usopp", "I-I am Captain Usopp! With 8000 followers!", COLOR_YELLOW);
 setDlg(&s->chapters[2].dialogues[1], "Luffy", "Cool! You're funny. Join my crew!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[2].dialogues[2], "Kaya", "Please... save our village from Kuro!", COLOR_WHITE);
 setDlg(&s->chapters[2].dialogues[3], "Usopp", "Even if I'm scared... I'll protect this village!", COLOR_YELLOW);
 setDlg(&s->chapters[2].dialogues[4], "Luffy", "That's the spirit! Let's go kick his butt!", LUFFY_SUN_GOLD);
 s->chapters[2].dialogue_count = 5;

 setDlg(&s->chapters[3].dialogues[0], "Sanji", "A cook fights with his hands, never his feet-- wait, reverse that.", LUFFY_ENERGY_ORANGE);
 setDlg(&s->chapters[3].dialogues[1], "Luffy", "Your food is amazing! Be my cook!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[3].dialogues[2], "Zeff", "That brat... he's got the same crazy dream as you once did.", COLOR_WHITE);
 setDlg(&s->chapters[3].dialogues[3], "Narrator", "Don Krieg's armada arrives at the Baratie!", COLOR_WHITE);
 setDlg(&s->chapters[3].dialogues[4], "Luffy", "Nobody messes with a place that feeds people!", LUFFY_SUN_GOLD);
 s->chapters[3].dialogue_count = 5;

 setDlg(&s->chapters[4].dialogues[0], "Nami", "I... I'm sorry. I had no choice... Arlong controls everything.", NARUTO_RASENGAN_BLUE);
 setDlg(&s->chapters[4].dialogues[1], "Luffy", "Nami... you're my navigator. That's all I need to know.", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[4].dialogues[2], "Arlong", "Humans are inferior! Your weakness disgusts me!", COLOR_BLUE);
 setDlg(&s->chapters[4].dialogues[3], "Luffy", "I don't know how to use swords or navigate... but I can beat YOU.", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[4].dialogues[4], "Narrator", "The room where Nami suffered... Luffy destroys it all.", COLOR_WHITE);
 setDlg(&s->chapters[4].dialogues[5], "Nami", "Luffy... help me.", NARUTO_RASENGAN_BLUE);
 setDlg(&s->chapters[4].dialogues[6], "Luffy", "OF COURSE I WILL!!!", LUFFY_SUN_GOLD);
 s->chapters[4].dialogue_count = 7;

 // Chapters 5-9 shorter
 for (int ch = 5; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Luffy", "Another adventure! Let's gooo!", LUFFY_SUN_GOLD);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Zoro", "This opponent is strong... but I have a dream too.", LEVI_SURVEY_GREEN);
   setDlg(&s->chapters[ch].dialogues[3], "Luffy", "I'll surpass every obstacle! That's my captain's duty!", LUFFY_SUN_GOLD);
   s->chapters[ch].dialogue_count = 4;
 }

 // Override with unique dialogues for later Luffy chapters
 // Ch 10 - Two Years Training
 setDlg(&s->chapters[9].dialogues[0], "Rayleigh", "To protect your crew, you must grow stronger, Luffy.", COLOR_WHITE);
 setDlg(&s->chapters[9].dialogues[1], "Luffy", "Teach me Haki! I'll protect everyone!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[9].dialogues[2], "Narrator", "Two years of training on Rusukaina Island...", COLOR_WHITE);
 setDlg(&s->chapters[9].dialogues[3], "Luffy", "I'm ready. My crew is counting on me!", LUFFY_SUN_GOLD);
 s->chapters[9].dialogue_count = 4;
 // Ch 13 - Dressrosa Liberation
 setDlg(&s->chapters[12].dialogues[0], "Law", "Doflamingo must be stopped. This is our alliance.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "Luffy", "Gear Fourth: BOUNDMAN!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[12].dialogues[2], "Narrator", "Luffy's new form shakes Dressrosa to its core!", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[3], "Luffy", "Gomu Gomu no... KING KONG GUN!", LUFFY_SUN_GOLD);
 s->chapters[12].dialogue_count = 4;
 // Ch 15 - Gear 5 Dawn
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "Drums echo across Wano... the rhythm of liberation.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Luffy", "Shishishi! I feel FREE! This is... GEAR FIVE!", LUFFY_SUN_GOLD);
 setDlg(&s->chapters[14].dialogues[2], "Narrator", "Sun God Nika awakens after 800 years!", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[3], "Kaido", "What IS this power?! His body is... rubber?!", COLOR_BLUE);
 setDlg(&s->chapters[14].dialogues[4], "Luffy", "I'll end this with JOY BOY'S WILL!", LUFFY_SUN_GOLD);
 s->chapters[14].dialogue_count = 5;

 // --- JINWOO ---
 s = &stories[THEME_SUNG_JINWOO];
 setDlg(&s->chapters[0].dialogues[0], "Jinwoo", "E-Rank Hunter Sung Jin-Woo... the weakest of all.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[0].dialogues[1], "Narrator", "A hidden dungeon within a dungeon appears...", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[2], "Jinwoo", "These statues... they're alive?!", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[0].dialogues[3], "System", "[You have been chosen as a Player.]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[0].dialogues[4], "Jinwoo", "A quest window? What is this... a game?", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[0].dialogues[5], "System", "[Daily Quest: Survival. Do you accept?]", JINWOO_ARISE_GLOW);
 s->chapters[0].dialogue_count = 6;

 setDlg(&s->chapters[1].dialogues[0], "System", "[Instance Dungeon Key acquired.]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[1].dialogues[1], "Jinwoo", "My stats are rising... I'm getting stronger.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[1].dialogues[2], "Narrator", "The dungeon shifts and transforms around him.", COLOR_WHITE);
 setDlg(&s->chapters[1].dialogues[3], "Jinwoo", "I won't be the weakest anymore. Never again.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[1].dialogues[4], "System", "[Level Up!]", JINWOO_ARISE_GLOW);
 s->chapters[1].dialogue_count = 5;

 setDlg(&s->chapters[2].dialogues[0], "System", "[Penalty Quest initiated. Survive 4 hours.]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[2].dialogues[1], "Jinwoo", "Poisonous snakes... in a closed room?!", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[2].dialogues[2], "Narrator", "With no escape, Jin-Woo fights for his life.", COLOR_WHITE);
 setDlg(&s->chapters[2].dialogues[3], "Jinwoo", "If I stop... I die. So I won't stop.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[2].dialogues[4], "System", "[Penalty Quest Complete. Reward: +3 to all stats.]", JINWOO_ARISE_GLOW);
 s->chapters[2].dialogue_count = 5;

 setDlg(&s->chapters[3].dialogues[0], "Narrator", "The Red Gate traps hunters inside a frozen hell.", COLOR_WHITE);
 setDlg(&s->chapters[3].dialogues[1], "Jinwoo", "Everyone is counting on me... I can't fail.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[3].dialogues[2], "Jinwoo", "Shadow extraction... ARISE!", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[3].dialogues[3], "Narrator", "From the fallen Ice Elf, a shadow rises...", COLOR_WHITE);
 setDlg(&s->chapters[3].dialogues[4], "Jinwoo", "Come forth, my soldiers.", JINWOO_MONARCH_PURPLE);
 s->chapters[3].dialogue_count = 5;

 setDlg(&s->chapters[4].dialogues[0], "Igris", "...", JINWOO_POWER_VIOLET);
 setDlg(&s->chapters[4].dialogues[1], "Jinwoo", "A knight in red armor... protecting a throne.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[4].dialogues[2], "Narrator", "Igris, the Blood-Red Commander, blocks the path.", COLOR_WHITE);
 setDlg(&s->chapters[4].dialogues[3], "Jinwoo", "You're strong. Perfect. You'll be my shadow.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[4].dialogues[4], "System", "[Shadow Extraction successful: IGRIS]", JINWOO_ARISE_GLOW);
 s->chapters[4].dialogue_count = 5;

 for (int ch = 5; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "System", "[New Quest Available.]", JINWOO_ARISE_GLOW);
   setDlg(&s->chapters[ch].dialogues[1], "Jinwoo", "Every shadow I collect makes me stronger.", JINWOO_MONARCH_PURPLE);
   setDlg(&s->chapters[ch].dialogues[2], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[3], "Jinwoo", "ARISE.", JINWOO_ARISE_GLOW);
   s->chapters[ch].dialogue_count = 4;
 }

 // Override with unique dialogues for later Jinwoo chapters
 // Ch 11 - S-Rank Hunter
 setDlg(&s->chapters[10].dialogues[0], "System", "[Player has reached S-Rank classification.]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[10].dialogues[1], "Jinwoo", "The Hunter Association can't explain my growth.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[10].dialogues[2], "Narrator", "Korea's newest S-Rank shakes the hunter world.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[3], "Jinwoo", "This is only the beginning.", JINWOO_MONARCH_PURPLE);
 s->chapters[10].dialogue_count = 4;
 // Ch 14 - Dragon Emperor
 setDlg(&s->chapters[13].dialogues[0], "Narrator", "The Dragon Emperor descends from the sky.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[1], "Jinwoo", "Another Monarch... I'll add you to my army.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[13].dialogues[2], "System", "[Shadow Army: 100,000 soldiers ready.]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[13].dialogues[3], "Jinwoo", "Come, my shadows. This is our war.", JINWOO_ARISE_GLOW);
 s->chapters[13].dialogue_count = 4;
 // Ch 15 - Final Battle: Antares
 setDlg(&s->chapters[14].dialogues[0], "Antares", "Shadow Monarch... at last we meet.", COLOR_RED);
 setDlg(&s->chapters[14].dialogues[1], "Jinwoo", "I've been waiting for you. My entire army stands behind me.", JINWOO_MONARCH_PURPLE);
 setDlg(&s->chapters[14].dialogues[2], "Narrator", "The final battle between Monarchs begins!", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[3], "System", "[Ultimate Shadow Army: ARISE]", JINWOO_ARISE_GLOW);
 setDlg(&s->chapters[14].dialogues[4], "Jinwoo", "I am the Shadow Monarch. This ends NOW.", JINWOO_ARISE_GLOW);
 s->chapters[14].dialogue_count = 5;

 // --- YUGO ---
 s = &stories[THEME_YUGO_WAKFU];
 setDlg(&s->chapters[0].dialogues[0], "Alibert", "Yugo, you're special. These portals... they're a gift.", YUGO_HAT_GOLD);
 setDlg(&s->chapters[0].dialogues[1], "Yugo", "But why can only I make them? Where do I come from?", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[0].dialogues[2], "Narrator", "A young Eliatrope discovers his portal powers.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[3], "Az", "*chirps excitedly, flapping tiny wings*", YUGO_PORTAL_GLOW);
 setDlg(&s->chapters[0].dialogues[4], "Yugo", "Az! My faithful tofu! Let's find our answers together!", YUGO_PORTAL_CYAN);
 s->chapters[0].dialogue_count = 5;

 setDlg(&s->chapters[1].dialogues[0], "Nox", "Time... I need more time... the Eliacube will give me everything.", YUGO_MYSTERY_BLUE);
 setDlg(&s->chapters[1].dialogues[1], "Yugo", "Who is that masked figure? His puppets are everywhere!", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[1].dialogues[2], "Ruel", "Kid, some things are best left alone. But fine, I'll tag along.", COLOR_GOLD);
 setDlg(&s->chapters[1].dialogues[3], "Amalia", "The Sadida kingdom will not fall to some clockmaker!", LEVI_SURVEY_GREEN);
 setDlg(&s->chapters[1].dialogues[4], "Yugo", "Together, we can stop him. I know it!", YUGO_PORTAL_CYAN);
 s->chapters[1].dialogue_count = 5;

 // Chapter 3 with Yugo choice point
 setDlg(&s->chapters[2].dialogues[0], "Adamai", "Yugo... we are Eliatropes. Dragons and portals are our legacy.", YUGO_ELIATROPE_TEAL);
 setDlg(&s->chapters[2].dialogues[1], "Yugo", "I feel something calling to me... two paths, two destinies.", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[2].dialogues[2], "Narrator", "The Brotherhood faces a crossroads...", COLOR_WHITE);
 setDlg(&s->chapters[2].dialogues[3], "Grougaloragran", "Choose wisely, young one. This shapes your future.", YUGO_HAT_GOLD);
 setDlg(&s->chapters[2].dialogues[4], "Narrator", "Embrace dragon power, or master the portals?", COLOR_WHITE,
   true, "Dragon Path", "Portal Path", 5, 5);
 s->chapters[2].dialogue_count = 5;

 for (int ch = 3; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Yugo", "Every portal takes me closer to the truth.", YUGO_PORTAL_CYAN);
   setDlg(&s->chapters[ch].dialogues[1], "Az", "*chirps and flutters protectively*", YUGO_PORTAL_GLOW);
   setDlg(&s->chapters[ch].dialogues[2], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[3], "Yugo", "I won't let anyone destroy the World of Twelve!", YUGO_PORTAL_CYAN);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Yugo chapters 11-15
 // Ch 11 - Eliatrope Council
 setDlg(&s->chapters[10].dialogues[0], "Narrator", "The six Eliatrope kings gather for the first time in centuries.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "Yugo", "I'm... one of the original six? A king?", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[10].dialogues[2], "Qilby", "Welcome home, brother. We have much to discuss.", YUGO_MYSTERY_BLUE);
 setDlg(&s->chapters[10].dialogues[3], "Yugo", "I may be a king, but my friends are my real family.", YUGO_PORTAL_CYAN);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - Zinit Awakening
 setDlg(&s->chapters[11].dialogues[0], "Narrator", "The ancient Eliatrope ship, the Zinit, stirs to life.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[1], "Adamai", "Yugo, the children are waking up. Our people live!", YUGO_ELIATROPE_TEAL);
 setDlg(&s->chapters[11].dialogues[2], "Yugo", "I can feel their Wakfu... hundreds of little lights!", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[11].dialogues[3], "Yugo", "I'll protect every single one of them. That's my duty as king.", YUGO_PORTAL_CYAN);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Dimension Walker
 setDlg(&s->chapters[12].dialogues[0], "Narrator", "Yugo's portals now pierce the walls between dimensions.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "Percedal", "Yugo! You teleported us to ANOTHER WORLD?!", COLOR_RED);
 setDlg(&s->chapters[12].dialogues[2], "Yugo", "Haha, sorry Dally! But the Krosmoz Guardian is here.", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[12].dialogues[3], "Yugo", "My portals can reach anywhere now. Any world. Any time.", YUGO_PORTAL_GLOW);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Brotherhood Reunited
 setDlg(&s->chapters[13].dialogues[0], "Amalia", "We've been through so much... but we're still together.", LEVI_SURVEY_GREEN);
 setDlg(&s->chapters[13].dialogues[1], "Ruel", "I'm getting too old for this! ...but fine, one more adventure.", COLOR_GOLD);
 setDlg(&s->chapters[13].dialogues[2], "Evangelyne", "The Brotherhood of the Tofu stands as one.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Yugo", "With my friends by my side, nothing can stop us!", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "The final challenge awaits the Brotherhood.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Master of Portals
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "Yugo channels the power of all six Dofus at once.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Yugo", "I can see it all... every dimension, every timeline!", YUGO_PORTAL_GLOW);
 setDlg(&s->chapters[14].dialogues[2], "Az", "*chirps fiercely, feathers blazing with Wakfu energy!*", YUGO_PORTAL_GLOW);
 setDlg(&s->chapters[14].dialogues[3], "Yugo", "I am the Master of Portals. Guardian of the Krosmoz!", YUGO_PORTAL_CYAN);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "The World of Twelve has found its eternal protector.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- NARUTO ---
 s = &stories[THEME_NARUTO_SAGE];
 setDlg(&s->chapters[0].dialogues[0], "Naruto", "One day, I'll be Hokage! Believe it!", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[0].dialogues[1], "Iruka", "Naruto... you've graduated. This headband is yours.", NARUTO_SAGE_GOLD);
 setDlg(&s->chapters[0].dialogues[2], "Narrator", "The Nine-Tails jinchuriki takes his first step.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[3], "Mizuki", "You're a monster, Naruto! The fox demon lives inside you!", COLOR_GRAY);
 setDlg(&s->chapters[0].dialogues[4], "Iruka", "Naruto is... one of my precious students!", NARUTO_SAGE_GOLD);
 setDlg(&s->chapters[0].dialogues[5], "Naruto", "Shadow Clone Jutsu!", NARUTO_CHAKRA_ORANGE);
 s->chapters[0].dialogue_count = 6;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Naruto", "I never go back on my word. That's my ninja way!", NARUTO_CHAKRA_ORANGE);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Kakashi", "Those who abandon their comrades are worse than scum.", COLOR_GRAY);
   setDlg(&s->chapters[ch].dialogues[3], "Naruto", "Rasengan!", NARUTO_RASENGAN_BLUE);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Naruto chapters 11-15
 // Ch 11 - Boruto Generation
 setDlg(&s->chapters[10].dialogues[0], "Naruto", "Being Hokage means protecting everyone in the village.", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[10].dialogues[1], "Boruto", "Dad, you're never home! What kind of Hokage are you?!", NARUTO_RASENGAN_BLUE);
 setDlg(&s->chapters[10].dialogues[2], "Naruto", "Boruto... the whole village is my family now.", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[10].dialogues[3], "Hinata", "Naruto-kun... our children have your spirit.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[4], "Naruto", "I'll protect them all. That's the Will of Fire!", NARUTO_SAGE_GOLD);
 s->chapters[10].dialogue_count = 5;
 // Ch 12 - Otsutsuki Threat
 setDlg(&s->chapters[11].dialogues[0], "Sasuke", "Naruto... they're coming. Beings from beyond our world.", COLOR_GRAY);
 setDlg(&s->chapters[11].dialogues[1], "Naruto", "The Otsutsuki? I thought Kaguya was the last!", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[11].dialogues[2], "Narrator", "A threat greater than any before descends upon Earth.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[3], "Naruto", "Sasuke, let's do this! Like old times, ya know!", NARUTO_CHAKRA_ORANGE);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Kurama's End
 setDlg(&s->chapters[12].dialogues[0], "Kurama", "Naruto... after all these years, I need to tell you something.", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[12].dialogues[1], "Naruto", "Kurama? What's wrong? You sound different...", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[12].dialogues[2], "Kurama", "This mode... Baryon Mode. It'll use up all my chakra. I'll die.", NARUTO_SAGE_GOLD);
 setDlg(&s->chapters[12].dialogues[3], "Naruto", "No... Kurama! You've been with me my whole life!", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[12].dialogues[4], "Kurama", "Heh... it wasn't so bad, having you as my partner.", NARUTO_SAGE_GOLD);
 s->chapters[12].dialogue_count = 5;
 // Ch 14 - Baryon Mode
 setDlg(&s->chapters[13].dialogues[0], "Narrator", "Baryon Mode activates. Naruto's power eclipses all limits.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[1], "Naruto", "This warmth... Kurama, lend me your strength one last time!", NARUTO_SAGE_GOLD);
 setDlg(&s->chapters[13].dialogues[2], "Narrator", "Fists of gold light rain down on the enemy.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Naruto", "For Kurama! For the village! RASENSHURIKEN!", NARUTO_RASENGAN_BLUE);
 s->chapters[13].dialogue_count = 4;
 // Ch 15 - Village Protector
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "Even without Kurama, Naruto stands as the Seventh Hokage.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Naruto", "I started as the kid nobody wanted. Now I carry their hopes.", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[14].dialogues[2], "Iruka", "Look at you, Naruto. You've surpassed every Hokage.", NARUTO_SAGE_GOLD);
 setDlg(&s->chapters[14].dialogues[3], "Naruto", "I'm Naruto Uzumaki, Seventh Hokage! Believe it!", NARUTO_CHAKRA_ORANGE);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "The boy who was once alone... became everyone's hero.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- GOKU ---
 s = &stories[THEME_GOKU_UI];
 setDlg(&s->chapters[0].dialogues[0], "Goku", "I'm Son Goku! I eat a lot and I love to fight!", GOKU_UI_SILVER);
 setDlg(&s->chapters[0].dialogues[1], "Raditz", "You're a Saiyan, Kakarot. You were sent to destroy this planet.", GOKU_GI_BLUE);
 setDlg(&s->chapters[0].dialogues[2], "Goku", "This is my home! These are my friends!", GOKU_UI_SILVER);
 setDlg(&s->chapters[0].dialogues[3], "Piccolo", "Goku... we'll need to work together. Just this once.", DEKU_HERO_GREEN);
 setDlg(&s->chapters[0].dialogues[4], "Goku", "Alright! Let's do this!", GOKU_UI_SILVER);
 s->chapters[0].dialogue_count = 5;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Goku", "I can feel it... a power beyond my limits!", GOKU_UI_SILVER);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Vegeta", "Kakarot... you always manage to surpass me.", GOKU_GI_BLUE);
   setDlg(&s->chapters[ch].dialogues[3], "Goku", "HAAAAAA! Kamehameha!", GOKU_KI_BLAST_BLUE);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Goku chapters 11-15
 // Ch 11 - Moro Arc
 setDlg(&s->chapters[10].dialogues[0], "Merus", "Goku, Ultra Instinct isn't about power. It's about letting go.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "Goku", "Letting go? But I love fighting! That IS my instinct!", GOKU_UI_SILVER);
 setDlg(&s->chapters[10].dialogues[2], "Moro", "I'll devour this planet's energy! No Saiyan can stop me!", COLOR_GREEN);
 setDlg(&s->chapters[10].dialogues[3], "Goku", "Heh, you haven't seen anything yet. Let's go!", GOKU_UI_SILVER);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - Granolah Saga
 setDlg(&s->chapters[11].dialogues[0], "Granolah", "Frieza destroyed my people! And you Saiyans helped him!", COLOR_RED);
 setDlg(&s->chapters[11].dialogues[1], "Goku", "I'm sorry for what happened. But I'm not my father!", GOKU_UI_SILVER);
 setDlg(&s->chapters[11].dialogues[2], "Vegeta", "Kakarot, this one fights with pure rage. Be careful.", GOKU_GI_BLUE);
 setDlg(&s->chapters[11].dialogues[3], "Goku", "I don't want to fight you, but I won't back down either!", GOKU_UI_SILVER);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Ultra Instinct Perfected
 setDlg(&s->chapters[12].dialogues[0], "Whis", "Goku, you've done it. True Ultra Instinct... with emotion.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "Goku", "I feel... everything. The wind, the stars, all of it!", GOKU_UI_SILVER);
 setDlg(&s->chapters[12].dialogues[2], "Narrator", "Silver hair flows in cosmic wind. The perfected form.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[3], "Goku", "This is MY Ultra Instinct! Not an angel's... MINE!", GOKU_UI_SILVER);
 setDlg(&s->chapters[12].dialogues[4], "Vegeta", "Tch. Show-off. But I have to admit... impressive.", GOKU_GI_BLUE);
 s->chapters[12].dialogue_count = 5;
 // Ch 14 - Beast Gohan Rising
 setDlg(&s->chapters[13].dialogues[0], "Gohan", "Father, I've been training again. I won't fall behind!", DEKU_HERO_GREEN);
 setDlg(&s->chapters[13].dialogues[1], "Goku", "That's my boy! Your power feels different... stronger!", GOKU_UI_SILVER);
 setDlg(&s->chapters[13].dialogues[2], "Narrator", "Beast Gohan unleashes power that rivals the gods!", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Goku", "Haha! The next generation might actually surpass us, Vegeta!", GOKU_UI_SILVER);
 s->chapters[13].dialogue_count = 4;
 // Ch 15 - Ultra Instinct Mastered
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "Son Goku stands at the pinnacle of mortal strength.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Beerus", "Hmph. You've surpassed my expectations, Saiyan.", COLOR_PURPLE);
 setDlg(&s->chapters[14].dialogues[2], "Goku", "There's always someone stronger out there! That's what makes it fun!", GOKU_UI_SILVER);
 setDlg(&s->chapters[14].dialogues[3], "Vegeta", "Don't get cocky, Kakarot. Our rivalry never ends.", GOKU_GI_BLUE);
 setDlg(&s->chapters[14].dialogues[4], "Goku", "Bring it on! Ka... me... ha... me... HAAAAA!", GOKU_KI_BLAST_BLUE);
 s->chapters[14].dialogue_count = 5;

 // --- TANJIRO ---
 s = &stories[THEME_TANJIRO_SUN];
 setDlg(&s->chapters[0].dialogues[0], "Tanjiro", "The smell of blood... no. My family!", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[0].dialogues[1], "Narrator", "Only Nezuko survived. But she has... changed.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[2], "Giyu", "If you want to save your sister, go to Mt. Sagiri.", TANJIRO_WATER_BLUE);
 setDlg(&s->chapters[0].dialogues[3], "Tanjiro", "Nezuko... I'll find a way to turn you back. I promise!", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[0].dialogues[4], "Urokodaki", "The Final Selection awaits. Survive... and become a Slayer.", COLOR_WHITE);
 s->chapters[0].dialogue_count = 5;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Tanjiro", "I can smell it... the demon is close.", TANJIRO_FIRE_ORANGE);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Zenitsu", "I'M GONNA DIE! WE'RE ALL GONNA DIE!", COLOR_YELLOW);
   setDlg(&s->chapters[ch].dialogues[3], "Tanjiro", "Hinokami Kagura... Dance of the Fire God!", TANJIRO_SUN_FIRE);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Tanjiro chapters 11-15
 // Ch 11 - Upper Moon Assault
 setDlg(&s->chapters[10].dialogues[0], "Narrator", "The Demon Slayer Corps storms Infinity Castle.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "Tanjiro", "Nezuko... wait for me. I'll end this tonight!", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[10].dialogues[2], "Inosuke", "YAHOO! Let's smash every demon in this creepy place!", COLOR_GRAY);
 setDlg(&s->chapters[10].dialogues[3], "Tanjiro", "Stay focused, Inosuke! The Upper Moons are close!", TANJIRO_FIRE_ORANGE);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - Demon King Confrontation
 setDlg(&s->chapters[11].dialogues[0], "Muzan", "You Demon Slayers... you're nothing but insects to me.", COLOR_RED);
 setDlg(&s->chapters[11].dialogues[1], "Tanjiro", "You killed my family! I'll never forgive you, Muzan!", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[11].dialogues[2], "Narrator", "The scent of blood and wisteria fills Infinity Castle.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[3], "Tanjiro", "Father... lend me strength. Sun Breathing, full power!", TANJIRO_SUN_FIRE);
 setDlg(&s->chapters[11].dialogues[4], "Giyu", "Tanjiro! I'll hold the line. You finish this!", TANJIRO_WATER_BLUE);
 s->chapters[11].dialogue_count = 5;
 // Ch 13 - Sun Breathing Mastery
 setDlg(&s->chapters[12].dialogues[0], "Narrator", "Tanjiro's earrings glow. The memory of Yoriichi flows through him.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "Tanjiro", "I can see it now... all thirteen forms, connected as one!", TANJIRO_SUN_FIRE);
 setDlg(&s->chapters[12].dialogues[2], "Yoriichi", "The sun breathing was never about strength. It was about kindness.", COLOR_GOLD);
 setDlg(&s->chapters[12].dialogues[3], "Tanjiro", "Hinokami Kagura... Thirteenth Form: SUN HALO DRAGON!", TANJIRO_SUN_FIRE);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Nezuko's Humanity
 setDlg(&s->chapters[13].dialogues[0], "Nezuko", "Onii-chan... I can... talk again?", COLOR_PINK);
 setDlg(&s->chapters[13].dialogues[1], "Tanjiro", "NEZUKO! You're... you're human again!", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[13].dialogues[2], "Narrator", "Tears stream down Tanjiro's face. His promise... fulfilled.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Tanjiro", "I kept my promise, Mom... Dad... everyone.", TANJIRO_FIRE_ORANGE);
 setDlg(&s->chapters[13].dialogues[4], "Nezuko", "Thank you, Tanjiro. For never giving up on me.", COLOR_PINK);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Dawn of Victory
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "Dawn breaks over the battlefield. Muzan screams as sunlight touches him.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Tanjiro", "Everyone... all the Hashira... all the fallen slayers... THIS IS FOR YOU!", TANJIRO_SUN_FIRE);
 setDlg(&s->chapters[14].dialogues[2], "Narrator", "The sun rises. The Demon King turns to ash.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[3], "Zenitsu", "W-we... we actually did it?! We're ALIVE?!", COLOR_YELLOW);
 setDlg(&s->chapters[14].dialogues[4], "Tanjiro", "The long night is over. Let the sun shine forever.", TANJIRO_SUN_FIRE);
 s->chapters[14].dialogue_count = 5;

 // --- GOJO ---
 s = &stories[THEME_GOJO_INFINITY];
 setDlg(&s->chapters[0].dialogues[0], "Gojo", "Throughout Heaven and Earth, I alone am the honored one.", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[0].dialogues[1], "Yuji", "Sensei... is that cursed spirit really dangerous?", SAITAMA_CAPE_RED);
 setDlg(&s->chapters[0].dialogues[2], "Gojo", "Nah. I'm here. Nothing is dangerous when I'm around~", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[0].dialogues[3], "Narrator", "The strongest sorcerer guides his students.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[4], "Gojo", "Infinity. Nothing touches me unless I allow it.", GOJO_SIX_EYES_BLUE);
 s->chapters[0].dialogue_count = 5;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Gojo", "Shall I show you what real power looks like?", GOJO_INFINITY_BLUE);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Megumi", "Gojo-sensei is reckless... but undeniably strong.", GOJO_VOID_BLACK);
   setDlg(&s->chapters[ch].dialogues[3], "Gojo", "Domain Expansion: Unlimited Void.", GOJO_DEEP_INFINITY);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Gojo chapters 11-15
 // Ch 11 - Prison Realm Escape
 setDlg(&s->chapters[10].dialogues[0], "Gojo", "You thought a little box could hold ME? How cute~", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[10].dialogues[1], "Narrator", "The Prison Realm shatters. Gojo Satoru returns.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[2], "Yuji", "G-Gojo-sensei! You're back!", SAITAMA_CAPE_RED);
 setDlg(&s->chapters[10].dialogues[3], "Gojo", "Miss me? Of course you did. Now then... who do I need to kill?", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[10].dialogues[4], "Megumi", "Sensei... a LOT has happened while you were gone.", GOJO_VOID_BLACK);
 s->chapters[10].dialogue_count = 5;
 // Ch 12 - Six Eyes Awakening
 setDlg(&s->chapters[11].dialogues[0], "Narrator", "The Six Eyes evolve beyond their known limits.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[1], "Gojo", "I can see... EVERYTHING. Every atom. Every cursed energy flow.", GOJO_SIX_EYES_BLUE);
 setDlg(&s->chapters[11].dialogues[2], "Gojo", "My Six Eyes are fully awakened. This is what it means to be the honored one.", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[11].dialogues[3], "Narrator", "No sorcerer in history has reached this level of perception.", COLOR_WHITE);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Unlimited Void Perfected
 setDlg(&s->chapters[12].dialogues[0], "Gojo", "My domain used to give you infinite information. Now? It gives you infinite NOTHING.", GOJO_DEEP_INFINITY);
 setDlg(&s->chapters[12].dialogues[1], "Narrator", "Unlimited Void expands. Time itself freezes within.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[2], "Gojo", "0.2 seconds? Nah. How about FOREVER? Haha~", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[12].dialogues[3], "Narrator", "Inside the domain, cursed spirits simply cease to exist.", COLOR_WHITE);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Sukuna Confrontation
 setDlg(&s->chapters[13].dialogues[0], "Sukuna", "Gojo Satoru. The so-called strongest. Entertain me.", COLOR_RED);
 setDlg(&s->chapters[13].dialogues[1], "Gojo", "Ryomen Sukuna. I've been WAITING for this fight~", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[13].dialogues[2], "Narrator", "Two domains clash! The sky splits! Shinjuku crumbles!", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Gojo", "You're the first person who's made me go all out. Respect.", GOJO_SIX_EYES_BLUE);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "The battle of the millennium shakes the foundations of jujutsu.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Strongest Sorcerer
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "When the dust settles, one man stands above all.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Gojo", "Throughout Heaven and Earth... I alone am the honored one.", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[14].dialogues[2], "Yuji", "Gojo-sensei... you really are the strongest.", SAITAMA_CAPE_RED);
 setDlg(&s->chapters[14].dialogues[3], "Gojo", "Of course! But you kids... you'll surpass me someday. And I can't wait~", GOJO_INFINITY_BLUE);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "Gojo Satoru. The Strongest. The one who changed the jujutsu world.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- LEVI ---
 s = &stories[THEME_LEVI_STRONGEST];
 setDlg(&s->chapters[0].dialogues[0], "Levi", "Tch. Another batch of brats who'll get themselves killed.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[0].dialogues[1], "Erwin", "Levi, humanity needs soldiers who can fight.", LEVI_SURVEY_GREEN);
 setDlg(&s->chapters[0].dialogues[2], "Narrator", "Humanity's Strongest Soldier. Captain Levi Ackerman.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[3], "Levi", "Give your hearts. That's all I ask.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[0].dialogues[4], "Narrator", "Beyond the walls, titans await...", COLOR_WHITE);
 s->chapters[0].dialogue_count = 5;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Levi", "I'll cut them down. Every last one.", LEVI_SILVER_BLADE);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Erwin", "ADVANCE! DEDICATE YOUR HEARTS!", LEVI_SURVEY_GREEN);
   setDlg(&s->chapters[ch].dialogues[3], "Levi", "No regrets.", LEVI_SILVER_BLADE);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Levi chapters 11-15
 // Ch 11 - Kenny's Legacy
 setDlg(&s->chapters[10].dialogues[0], "Narrator", "Kenny Ackerman. The man who raised Levi in the Underground.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "Levi", "Kenny... you taught me to survive. But not how to live.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[10].dialogues[2], "Kenny", "Everyone needs to be drunk on somethin' to keep pushin'. What's yours, Levi?", COLOR_GRAY);
 setDlg(&s->chapters[10].dialogues[3], "Levi", "Tch. My squad. Their lives. That's what I'm drunk on.", LEVI_SILVER_BLADE);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - Ackerman Bloodline
 setDlg(&s->chapters[11].dialogues[0], "Hange", "The Ackerman bloodline... you're not entirely human, Levi.", LEVI_SURVEY_GREEN);
 setDlg(&s->chapters[11].dialogues[1], "Levi", "I don't care what I am. I fight for humanity. Period.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[11].dialogues[2], "Narrator", "Ackerman power surges. Levi moves faster than the eye can follow.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[3], "Levi", "If this blood makes me stronger... then I'll use it to protect them.", LEVI_SILVER_BLADE);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Beyond the Walls
 setDlg(&s->chapters[12].dialogues[0], "Narrator", "The ocean stretches before them. A world beyond the walls.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "Armin", "It's real... the sea! Just like the books said!", NARUTO_RASENGAN_BLUE);
 setDlg(&s->chapters[12].dialogues[2], "Levi", "...It's just salty water. But the brats seem happy.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[12].dialogues[3], "Narrator", "For the first time, Levi allows himself a small smile.", COLOR_WHITE);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Freedom's Cost
 setDlg(&s->chapters[13].dialogues[0], "Levi", "Erwin... I chose Armin. I hope you understand.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[13].dialogues[1], "Narrator", "Scars cover Levi's body. Every wound, a comrade lost.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[2], "Levi", "I've lost Isabel. Furlan. Erwin. My entire squad. Twice.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[13].dialogues[3], "Levi", "But I'll keep fighting. That's the promise I made to them.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "The weight of loss fuels humanity's strongest soldier.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Humanity's Last Hope
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "The Rumbling threatens to flatten the world.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Levi", "I made a promise to Erwin. To kill the Beast Titan.", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[14].dialogues[2], "Narrator", "Blades spinning, Levi descends like death itself.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[3], "Levi", "Zeke... this is for them. For ALL of them. Give your hearts!", LEVI_SILVER_BLADE);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "Captain Levi Ackerman. Humanity's Strongest. Forever.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- SAITAMA ---
 s = &stories[THEME_SAITAMA_OPM];
 setDlg(&s->chapters[0].dialogues[0], "Saitama", "I became a hero for fun.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[0].dialogues[1], "Narrator", "One Punch Man. The hero nobody believes in.", COLOR_WHITE);
 setDlg(&s->chapters[0].dialogues[2], "Saitama", "100 push-ups, 100 sit-ups, 100 squats, 10km run. Every day.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[0].dialogues[3], "Saitama", "I'm just a guy who's a hero for fun.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[0].dialogues[4], "Narrator", "But the cost... was all his hair.", COLOR_WHITE);
 s->chapters[0].dialogue_count = 5;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Saitama", "OK.", SAITAMA_HERO_YELLOW);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Genos", "Sensei! Let me record your training methods!", SAITAMA_GOLDEN_PUNCH);
   setDlg(&s->chapters[ch].dialogues[3], "Saitama", "One punch is all I need.", SAITAMA_HERO_YELLOW);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Saitama chapters 11-15
 // Ch 11 - Hero Name Fame
 setDlg(&s->chapters[10].dialogues[0], "King", "Saitama... people still think I defeated those monsters.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[10].dialogues[1], "Saitama", "It's fine. I don't do it for fame. I just want a good fight.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[10].dialogues[2], "Genos", "Sensei deserves S-Class! The Hero Association is blind!", SAITAMA_GOLDEN_PUNCH);
 setDlg(&s->chapters[10].dialogues[3], "Saitama", "Meh. Rankings don't change who you are. ...Is there a sale at the supermarket?", SAITAMA_HERO_YELLOW);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - S-Class Recognition
 setDlg(&s->chapters[11].dialogues[0], "Narrator", "The Hero Association finally notices Saitama's true power.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[1], "Sweet Mask", "Impossible... Class B beat THAT in one punch?!", COLOR_BLUE);
 setDlg(&s->chapters[11].dialogues[2], "Saitama", "Can I go home now? I have coupons that expire today.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[11].dialogues[3], "Genos", "This is the man I call Master. REMEMBER HIS NAME!", SAITAMA_GOLDEN_PUNCH);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Monster King
 setDlg(&s->chapters[12].dialogues[0], "Orochi", "I am the Monster King! The pinnacle of evolution!", COLOR_RED);
 setDlg(&s->chapters[12].dialogues[1], "Saitama", "Cool. Anyway... *punches*", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[12].dialogues[2], "Narrator", "...The Monster King evaporates.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[3], "Saitama", "That was it? Man, I was hoping for at least two punches.", SAITAMA_HERO_YELLOW);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Serious Series
 setDlg(&s->chapters[13].dialogues[0], "Narrator", "For the first time in years, Saitama feels... something.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[1], "Saitama", "Huh. My heart is beating fast. Is this... excitement?", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[13].dialogues[2], "Narrator", "His eyes sharpen. His fist clenches. This is SERIOUS.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "Saitama", "Serious Series... Serious Punch.", SAITAMA_GOLDEN_PUNCH);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "The sky splits in half. The Earth trembles.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - One Punch Legend
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "People whisper about a bald hero who ends every fight in one blow.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Saitama", "I just wanted a good fight. Instead I got... this.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[14].dialogues[2], "Genos", "Sensei, you've saved the world countless times. You ARE a legend.", SAITAMA_GOLDEN_PUNCH);
 setDlg(&s->chapters[14].dialogues[3], "Saitama", "A legend? Nah. Just a guy who's a hero for fun.", SAITAMA_HERO_YELLOW);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "One Punch Man. The strongest. The loneliest. The legend.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- DEKU ---
 s = &stories[THEME_DEKU_PLUSULTRA];
 setDlg(&s->chapters[0].dialogues[0], "Deku", "I was born without a Quirk. In a world where 80% have powers...", DEKU_HERO_GREEN);
 setDlg(&s->chapters[0].dialogues[1], "All Might", "You CAN become a hero! I deem you worthy!", DEKU_ALLMIGHT_GOLD);
 setDlg(&s->chapters[0].dialogues[2], "Deku", "Eat... this hair?!", DEKU_HERO_GREEN);
 setDlg(&s->chapters[0].dialogues[3], "All Might", "One For All! The power passed from hero to hero!", DEKU_ALLMIGHT_GOLD);
 setDlg(&s->chapters[0].dialogues[4], "Deku", "SMAAASH! ...ow, my arm.", DEKU_OFA_LIGHTNING);
 setDlg(&s->chapters[0].dialogues[5], "Narrator", "The quirkless boy begins his path to become the greatest hero.", COLOR_WHITE);
 s->chapters[0].dialogue_count = 6;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "Deku", "I have to give it my all... PLUS ULTRA!", DEKU_HERO_GREEN);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Bakugo", "GET OUT OF MY WAY, DEKU!", COLOR_ORANGE);
   setDlg(&s->chapters[ch].dialogues[3], "Deku", "Full Cowl: 100%! Delaware Detroit SMASH!", DEKU_FULL_COWL);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique Deku chapters 11-15
 // Ch 11 - One For All Mastery
 setDlg(&s->chapters[10].dialogues[0], "Narrator", "The vestiges of One For All speak to Izuku in his mind.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "First OFA User", "You are the ninth and final holder. All our power is yours.", COLOR_WHITE);
 setDlg(&s->chapters[10].dialogues[2], "Deku", "Six quirks... Float, Danger Sense, Smokescreen, Blackwhip, Fa Jin, Gearshift!", DEKU_OFA_LIGHTNING);
 setDlg(&s->chapters[10].dialogues[3], "Deku", "I'll master them all! That's what it means to be the successor!", DEKU_HERO_GREEN);
 s->chapters[10].dialogue_count = 4;
 // Ch 12 - Shigaraki Confrontation
 setDlg(&s->chapters[11].dialogues[0], "Shigaraki", "Midoriya... I'll take One For All and destroy everything.", COLOR_GRAY);
 setDlg(&s->chapters[11].dialogues[1], "Deku", "Shigaraki... I saw your memories. You were just a scared kid!", DEKU_HERO_GREEN);
 setDlg(&s->chapters[11].dialogues[2], "Narrator", "A hero who wants to save even his enemy. That's Izuku Midoriya.", COLOR_WHITE);
 setDlg(&s->chapters[11].dialogues[3], "Deku", "I'll save you too, Shigaraki! That's what heroes DO!", DEKU_HERO_GREEN);
 setDlg(&s->chapters[11].dialogues[4], "Bakugo", "Tch. Only you would try to save the final boss, nerd.", COLOR_ORANGE);
 s->chapters[11].dialogue_count = 5;
 // Ch 13 - All Might's Legacy
 setDlg(&s->chapters[12].dialogues[0], "All Might", "Young Midoriya... you've far surpassed what I ever was.", DEKU_ALLMIGHT_GOLD);
 setDlg(&s->chapters[12].dialogues[1], "Deku", "All Might... everything I am is because of you.", DEKU_HERO_GREEN);
 setDlg(&s->chapters[12].dialogues[2], "Narrator", "The torch passes fully. Deku carries the Symbol of Peace.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[3], "All Might", "I am here... in YOU, young Midoriya. Always.", DEKU_ALLMIGHT_GOLD);
 s->chapters[12].dialogue_count = 4;
 // Ch 14 - Quirk Singularity
 setDlg(&s->chapters[13].dialogues[0], "Narrator", "One For All's power reaches the theoretical singularity.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[1], "Deku", "My body is breaking... but I can't stop now! Not when everyone is counting on me!", DEKU_OFA_LIGHTNING);
 setDlg(&s->chapters[13].dialogues[2], "Uraraka", "Deku-kun! Please... don't destroy yourself!", COLOR_PINK);
 setDlg(&s->chapters[13].dialogues[3], "Deku", "Gearshift plus Fa Jin plus 120%... UNITED STATES OF SMASH!", DEKU_FULL_COWL);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "The sky cracks with green lightning. This is Plus Ultra.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Symbol of Peace
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "The war is over. Izuku Midoriya stands in the ruins.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "Deku", "I was born quirkless. Everyone said I couldn't be a hero.", DEKU_HERO_GREEN);
 setDlg(&s->chapters[14].dialogues[2], "Bakugo", "...You did good, Izuku. Don't let it go to your head.", COLOR_ORANGE);
 setDlg(&s->chapters[14].dialogues[3], "Deku", "This is my hero academia. Every lesson... every friend... led me here.", DEKU_HERO_GREEN);
 setDlg(&s->chapters[14].dialogues[4], "Narrator", "Izuku Midoriya. The greatest hero. The new Symbol of Peace.", COLOR_WHITE);
 s->chapters[14].dialogue_count = 5;

 // --- BOBOIBOY ---
 s = &stories[THEME_BOBOIBOY];
 setDlg(&s->chapters[0].dialogues[0], "Ochobot", "BoBoiBoy! I'm giving you elemental powers!", BBB_OCHOBOT_WHITE);
 setDlg(&s->chapters[0].dialogues[1], "BoBoiBoy", "Woah! Lightning, Wind AND Earth?! Awesome!", BBB_BAND_ORANGE);
 setDlg(&s->chapters[0].dialogues[2], "Adu Du", "That Power Sphera belongs to ME! Probe, attack!", BBB_LEAF_GREEN);
 setDlg(&s->chapters[0].dialogues[3], "BoBoiBoy", "BoBoiBoy Lightning! Hiyaaah!", BBB_LIGHTNING_YELLOW);
 setDlg(&s->chapters[0].dialogues[4], "Gopal", "Dey! Can you teach me too?!", COLOR_YELLOW);
 setDlg(&s->chapters[0].dialogues[5], "BoBoiBoy", "Let's protect our friends! Terbaik!", BBB_BAND_ORANGE);
 s->chapters[0].dialogue_count = 6;

 for (int ch = 1; ch < 15; ch++) {
   setDlg(&s->chapters[ch].dialogues[0], "BoBoiBoy", "Elemental Power! BoBoiBoy... SPLIT!", BBB_BAND_ORANGE);
   setDlg(&s->chapters[ch].dialogues[1], "Narrator", s->chapters[ch].title, COLOR_WHITE);
   setDlg(&s->chapters[ch].dialogues[2], "Ochobot", "Be careful, BoBoiBoy! This enemy is strong!", BBB_OCHOBOT_WHITE);
   setDlg(&s->chapters[ch].dialogues[3], "BoBoiBoy", "BoBoiBoy Galaxy! TERBAIK!", BBB_SUPRA_GOLD);
   s->chapters[ch].dialogue_count = 4;
 }

 // Unique BoBoiBoy chapters 11-15
 // Ch 11 - TAPOPS Academy
 setDlg(&s->chapters[10].dialogues[0], "Koko Ci", "BoBoiBoy, TAPOPS has a new mission. Galaxy-level threat.", BBB_OCHOBOT_WHITE);
 setDlg(&s->chapters[10].dialogues[1], "BoBoiBoy", "Another galaxy mission? Awesome! Terbaik!", BBB_BAND_ORANGE);
 setDlg(&s->chapters[10].dialogues[2], "Gopal", "Dey! Why do we always get the dangerous ones?!", COLOR_YELLOW);
 setDlg(&s->chapters[10].dialogues[3], "Yaya", "Because we're the best team in TAPOPS, Gopal!", COLOR_PINK);
 setDlg(&s->chapters[10].dialogues[4], "BoBoiBoy", "Let's go, team! Elemental power, activate!", BBB_BAND_ORANGE);
 s->chapters[10].dialogue_count = 5;
 // Ch 12 - Captain Kaizo's Trial
 setDlg(&s->chapters[11].dialogues[0], "Captain Kaizo", "BoBoiBoy. If you want to master your powers, you must defeat ME.", COLOR_PURPLE);
 setDlg(&s->chapters[11].dialogues[1], "Fang", "Abang! Don't hurt my friend too badly!", COLOR_PURPLE);
 setDlg(&s->chapters[11].dialogues[2], "BoBoiBoy", "I won't back down! BoBoiBoy Triple Split!", BBB_BAND_ORANGE);
 setDlg(&s->chapters[11].dialogues[3], "Captain Kaizo", "Impressive. You've grown, young one.", COLOR_PURPLE);
 s->chapters[11].dialogue_count = 4;
 // Ch 13 - Seven Elements Mastery
 setDlg(&s->chapters[12].dialogues[0], "Ochobot", "BoBoiBoy! All seven elements are resonating at once!", BBB_OCHOBOT_WHITE);
 setDlg(&s->chapters[12].dialogues[1], "BoBoiBoy", "Lightning! Wind! Earth! Fire! Water! Leaf! Light!", BBB_SUPRA_GOLD);
 setDlg(&s->chapters[12].dialogues[2], "Narrator", "Seven BoBoiBoys stand together. Seven elements, one will.", COLOR_WHITE);
 setDlg(&s->chapters[12].dialogues[3], "BoBoiBoy", "I can control them ALL now! SEVEN SPLIT!", BBB_BAND_ORANGE);
 setDlg(&s->chapters[12].dialogues[4], "Ying", "Wah! He's even faster than me now!", NARUTO_RASENGAN_BLUE);
 s->chapters[12].dialogue_count = 5;
 // Ch 14 - Kuasa Tujuh
 setDlg(&s->chapters[13].dialogues[0], "Narrator", "The seven elements begin to FUSE into one.", COLOR_WHITE);
 setDlg(&s->chapters[13].dialogues[1], "BoBoiBoy", "This power... it's all seven elements combined!", BBB_SUPRA_GOLD);
 setDlg(&s->chapters[13].dialogues[2], "Ochobot", "Kuasa Tujuh! The Power of Seven! It's real!", BBB_OCHOBOT_WHITE);
 setDlg(&s->chapters[13].dialogues[3], "BoBoiBoy", "BoBoiBoy Kuasa Tujuh! ULTIMATE ELEMENTAL POWER!", BBB_SUPRA_GOLD);
 setDlg(&s->chapters[13].dialogues[4], "Narrator", "Golden light engulfs the battlefield. Nothing can withstand this.", COLOR_WHITE);
 s->chapters[13].dialogue_count = 5;
 // Ch 15 - Ultimate Elemental Hero
 setDlg(&s->chapters[14].dialogues[0], "Narrator", "BoBoiBoy stands as the strongest elemental wielder in the galaxy.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[1], "BoBoiBoy", "I started as a kid visiting Tok Aba's cocoa shop.", BBB_BAND_ORANGE);
 setDlg(&s->chapters[14].dialogues[2], "Tok Aba", "BoBoiBoy, I'm so proud of you. Your parents would be too.", COLOR_WHITE);
 setDlg(&s->chapters[14].dialogues[3], "BoBoiBoy", "I'll protect everyone! My friends, my family, the GALAXY!", BBB_SUPRA_GOLD);
 setDlg(&s->chapters[14].dialogues[4], "All Friends", "TERBAIK!!!", BBB_SUPRA_GOLD);
 setDlg(&s->chapters[14].dialogues[5], "Narrator", "BoBoiBoy. The Ultimate Elemental Hero. TERBAIK!", COLOR_WHITE);
 s->chapters[14].dialogue_count = 6;
}

// =============================================================================
// BOSS SPRITE DRAWING
// =============================================================================
void drawBossSprite(int x, int y, StoryBoss* boss, int size, int anim_frame) {
 int bob = (anim_frame % 2) * 2 - 1;
 y += bob;

 switch (boss->visual_type) {
   case BOSS_HUMANOID: {
     // Head
     gfx->fillCircle(x, y - size/2, size/4, boss->boss_color);
     // Body
     gfx->fillRect(x - size/4, y - size/4, size/2, size/2, boss->boss_color);
     // Arms
     gfx->fillRect(x - size/2, y - size/6, size/5, size/8, boss->boss_accent);
     gfx->fillRect(x + size/3, y - size/6, size/5, size/8, boss->boss_accent);
     // Legs
     gfx->fillRect(x - size/5, y + size/4, size/7, size/4, boss->boss_color);
     gfx->fillRect(x + size/10, y + size/4, size/7, size/4, boss->boss_color);
     // Eyes (menacing)
     gfx->fillRect(x - size/8, y - size/2 - 2, size/10, size/12, COLOR_RED);
     gfx->fillRect(x + size/12, y - size/2 - 2, size/10, size/12, COLOR_RED);
     // Aura glow
     gfx->drawCircle(x, y, size/2 + 5, boss->boss_accent);
     break;
   }
   case BOSS_BEAST: {
     // Main body (hunched)
     gfx->fillCircle(x, y - size/6, size/3, boss->boss_color);
     gfx->fillCircle(x, y + size/6, size/3 + 4, boss->boss_color);
     // Head with snout
     gfx->fillCircle(x, y - size/2 + 5, size/4, boss->boss_color);
     gfx->fillTriangle(x, y - size/2 - size/4, x - size/6, y - size/3, x + size/6, y - size/3, boss->boss_color);
     // Eyes
     gfx->fillCircle(x - size/8, y - size/2, 3, COLOR_RED);
     gfx->fillCircle(x + size/8, y - size/2, 3, COLOR_RED);
     // Teeth
     for (int t = -2; t <= 2; t++) {
       gfx->fillTriangle(x + t*5, y - size/3, x + t*5 - 2, y - size/3 + 5, x + t*5 + 2, y - size/3 + 5, COLOR_WHITE);
     }
     // Claws
     gfx->fillTriangle(x - size/3, y + size/4, x - size/3 - 8, y + size/2, x - size/3 + 4, y + size/2, boss->boss_accent);
     gfx->fillTriangle(x + size/3, y + size/4, x + size/3 - 4, y + size/2, x + size/3 + 8, y + size/2, boss->boss_accent);
     break;
   }
   case BOSS_GIANT: {
     // Massive body
     gfx->fillRect(x - size/3, y - size/3, size*2/3, size*2/3, boss->boss_color);
     // Small head
     gfx->fillCircle(x, y - size/2 + 5, size/5, boss->boss_color);
     // Eyes
     gfx->fillRect(x - size/8, y - size/2 + 2, size/6, size/10, boss->boss_accent);
     // Thick arms
     gfx->fillRect(x - size/2 - size/6, y - size/4, size/5, size/2, boss->boss_color);
     gfx->fillRect(x + size/3, y - size/4, size/5, size/2, boss->boss_color);
     // Fists
     gfx->fillCircle(x - size/2 - size/10, y + size/4, size/8, boss->boss_accent);
     gfx->fillCircle(x + size/3 + size/10, y + size/4, size/8, boss->boss_accent);
     // Ground crack
     gfx->drawLine(x - size/2, y + size/3 + 5, x + size/2, y + size/3 + 5, boss->boss_accent);
     gfx->drawLine(x - size/3, y + size/3 + 8, x + size/3, y + size/3 + 8, boss->boss_accent);
     break;
   }
   case BOSS_ELEMENTAL: {
     // Swirling elemental core
     for (int r = size/2; r > 5; r -= 4) {
       uint16_t col = (r % 8 < 4) ? boss->boss_color : boss->boss_accent;
       gfx->drawCircle(x, y, r, col);
     }
     gfx->fillCircle(x, y, size/6, COLOR_WHITE);
     // Elemental tendrils
     for (int i = 0; i < 6; i++) {
       float angle = (i * 60 + anim_frame * 30) * PI / 180.0;
       int tx = x + cos(angle) * size/2;
       int ty = y + sin(angle) * size/2;
       gfx->drawLine(x, y, tx, ty, boss->boss_accent);
       gfx->fillCircle(tx, ty, 4, boss->boss_accent);
     }
     // Eyes in core
     gfx->fillCircle(x - 5, y - 3, 3, boss->boss_color);
     gfx->fillCircle(x + 5, y - 3, 3, boss->boss_color);
     break;
   }
   case BOSS_DEMON: {
     // Dark body
     gfx->fillCircle(x, y, size/3, boss->boss_color);
     // Horns
     gfx->fillTriangle(x - size/4, y - size/3, x - size/6, y - size/2 - size/4, x - size/8, y - size/3, boss->boss_accent);
     gfx->fillTriangle(x + size/4, y - size/3, x + size/6, y - size/2 - size/4, x + size/8, y - size/3, boss->boss_accent);
     // Glowing eyes
     gfx->fillCircle(x - size/8, y - size/8, 5, COLOR_RED);
     gfx->fillCircle(x + size/8, y - size/8, 5, COLOR_RED);
     gfx->fillCircle(x - size/8, y - size/8, 2, COLOR_WHITE);
     gfx->fillCircle(x + size/8, y - size/8, 2, COLOR_WHITE);
     // Dark wings
     gfx->fillTriangle(x - size/3, y, x - size/2 - size/4, y - size/4, x - size/3, y + size/4, boss->boss_color);
     gfx->fillTriangle(x + size/3, y, x + size/2 + size/4, y - size/4, x + size/3, y + size/4, boss->boss_color);
     // Mouth
     gfx->drawLine(x - size/6, y + size/8, x + size/6, y + size/8, COLOR_RED);
     // Dark aura
     gfx->drawCircle(x, y, size/2 + 3, boss->boss_accent);
     gfx->drawCircle(x, y, size/2 + 6, boss->boss_color);
     break;
   }
 }
}

// =============================================================================
// INITIALIZATION
// =============================================================================
void initStorySystem() {
 story_system.current_story = nullptr;
 story_system.current_dialogue_index = 0;
 story_system.in_story_mode = false;
 story_system.in_boss_battle = false;
 story_system.showing_rewards = false;
 story_system.player_hp = 100;
 story_system.player_max_hp = 100;
 story_system.boss_hp = 0;
 story_system.boss_max_hp = 0;
 story_system.last_event_check_day = -1;
 story_system.nvs_initialized = false;
 
 // Initialize notification system
 story_system.notification.isActive = false;
 story_system.notification.isPending = false;
 story_system.notification.chapterUnlocked = 0;
 story_system.notification.startTime = 0;
 story_system.notification.userDismissed = false;
 story_system.notification.userAccepted = false;
 story_system.lastCheckedLevel = 0;

 for (int i = 0; i < THEME_COUNT; i++) {
   stories[i].character = (ThemeType)i;
   stories[i].story_name = STORY_NAMES[i];
   stories[i].current_chapter = 0;
   stories[i].chapters_completed = 0;
   stories[i].story_completed = false;
   stories[i].yugo_path = YUGO_PATH_UNDECIDED;

   for (int j = 0; j < MAX_CHAPTERS_PER_CHARACTER; j++) {
     stories[i].chapters[j].chapter_number = j + 1;
     stories[i].chapters[j].title = CHAPTER_TITLES[i][j];
     stories[i].chapters[j].level_required = CHAPTER_LEVELS[j];
     stories[i].chapters[j].dialogue_count = 4;
     stories[i].chapters[j].boss = ALL_BOSSES[i][j];
     stories[i].chapters[j].completed = false;
     stories[i].chapters[j].boss_defeated = false;
     stories[i].chapters[j].rewards_claimed = false;

     // Default dialogues
     for (int k = 0; k < MAX_DIALOGUE_SCREENS; k++) {
       stories[i].chapters[j].dialogues[k] = {"Narrator", "The adventure continues...", COLOR_WHITE, false, "", "", 0, 0};
     }
   }
 }

 // Load actual dialogue content
 initStoryDialogues();

 // Daily events
 story_system.daily_events[0] = {"Morning Training", "Complete exercises", 50, 6, 10, false, THEME_COUNT};
 story_system.daily_events[1] = {"Afternoon Quest", "Help the villagers", 75, 12, 16, false, THEME_COUNT};
 story_system.daily_events[2] = {"Evening Battle", "Face the darkness", 100, 18, 22, false, THEME_COUNT};
 story_system.daily_events[3] = {"Midnight Mystery", "Uncover secrets", 150, 0, 4, false, THEME_COUNT};

 if (story_system.prefs.begin(STORY_NVS_NAMESPACE, false)) {
   story_system.nvs_initialized = true;
   loadStoryProgress();
 }
}

// =============================================================================
// NVS SAVE/LOAD (same as before)
// =============================================================================
void saveStoryProgress() {
 if (!story_system.nvs_initialized) return;
 for (int i = 0; i < THEME_COUNT; i++) saveStoryProgressForCharacter((ThemeType)i);
 story_system.prefs.putInt("last_event_day", story_system.last_event_check_day);
 for (int i = 0; i < 4; i++) {
   char key[20]; snprintf(key, sizeof(key), "event_%d_done", i);
   story_system.prefs.putBool(key, story_system.daily_events[i].completed_today);
 }
 story_system.prefs.putBool("initialized", true);
}

void saveStoryProgressForCharacter(ThemeType character) {
 if (!story_system.nvs_initialized || character >= THEME_COUNT) return;
 CharacterStory* st = &stories[character]; char key[32];
 getCharKey(character, "chapter", key, sizeof(key)); story_system.prefs.putInt(key, st->current_chapter);
 getCharKey(character, "completed", key, sizeof(key)); story_system.prefs.putInt(key, st->chapters_completed);
 getCharKey(character, "done", key, sizeof(key)); story_system.prefs.putBool(key, st->story_completed);
 if (character == THEME_YUGO_WAKFU) story_system.prefs.putInt("yugo_path", (int)st->yugo_path);
 for (int j = 0; j < MAX_CHAPTERS_PER_CHARACTER; j++) {
   getChKey(character, j, "done", key, sizeof(key)); story_system.prefs.putBool(key, st->chapters[j].completed);
   getChKey(character, j, "boss", key, sizeof(key)); story_system.prefs.putBool(key, st->chapters[j].boss_defeated);
   getChKey(character, j, "reward", key, sizeof(key)); story_system.prefs.putBool(key, st->chapters[j].rewards_claimed);
 }
 // FIX: Also set initialized flag so loadStoryProgress() works after reboot
 story_system.prefs.putBool("initialized", true);
 Serial.printf("[Story] Saved progress for character %d (completed: %d)\n", (int)character, st->chapters_completed);
}

void loadStoryProgress() {
 if (!story_system.nvs_initialized) return;
 // FIX: Removed "initialized" check - if NVS has no data, getBool/getInt
 // return defaults anyway. The old check caused chapter progress to be lost
 // because saveStoryProgressForCharacter() didn't set the flag.
 for (int i = 0; i < THEME_COUNT; i++) loadStoryProgressForCharacter((ThemeType)i);
 story_system.last_event_check_day = story_system.prefs.getInt("last_event_day", -1);
 for (int i = 0; i < 4; i++) {
   char key[20]; snprintf(key, sizeof(key), "event_%d_done", i);
   story_system.daily_events[i].completed_today = story_system.prefs.getBool(key, false);
 }
}

void loadStoryProgressForCharacter(ThemeType character) {
 if (!story_system.nvs_initialized || character >= THEME_COUNT) return;
 CharacterStory* st = &stories[character]; char key[32];
 getCharKey(character, "chapter", key, sizeof(key)); st->current_chapter = story_system.prefs.getInt(key, 0);
 getCharKey(character, "completed", key, sizeof(key)); st->chapters_completed = story_system.prefs.getInt(key, 0);
 getCharKey(character, "done", key, sizeof(key)); st->story_completed = story_system.prefs.getBool(key, false);
 if (character == THEME_YUGO_WAKFU) {
   YugoStoryPath p = (YugoStoryPath)story_system.prefs.getInt("yugo_path", YUGO_PATH_UNDECIDED);
   st->yugo_path = p; if (p != YUGO_PATH_UNDECIDED) loadYugoPathStory(p);
 }
 for (int j = 0; j < MAX_CHAPTERS_PER_CHARACTER; j++) {
   getChKey(character, j, "done", key, sizeof(key)); st->chapters[j].completed = story_system.prefs.getBool(key, false);
   getChKey(character, j, "boss", key, sizeof(key)); st->chapters[j].boss_defeated = story_system.prefs.getBool(key, false);
   getChKey(character, j, "reward", key, sizeof(key)); st->chapters[j].rewards_claimed = story_system.prefs.getBool(key, false);
 }
}

void clearAllStoryProgress() {
 if (!story_system.nvs_initialized) return;
 story_system.prefs.clear();
 for (int i = 0; i < THEME_COUNT; i++) {
   stories[i].current_chapter = 0; stories[i].chapters_completed = 0;
   stories[i].story_completed = false; stories[i].yugo_path = YUGO_PATH_UNDECIDED;
   for (int j = 0; j < MAX_CHAPTERS_PER_CHARACTER; j++) {
     stories[i].chapters[j].completed = false; stories[i].chapters[j].boss_defeated = false;
     stories[i].chapters[j].rewards_claimed = false;
   }
 }
}

// =============================================================================
// STORY MANAGEMENT
// =============================================================================
CharacterStory* getCharacterStory(ThemeType t) { return (t < THEME_COUNT) ? &stories[t] : nullptr; }
void setCurrentStory(ThemeType t) {
 if (t < THEME_COUNT) {
   story_system.current_story = &stories[t];
   story_system.in_story_mode = true;
   // Sync lastCheckedLevel so we don't falsely trigger unlock notifications
   CharacterXPData* xp = getCurrentCharacterXP();
   story_system.lastCheckedLevel = xp ? xp->level : 1;
 }
}
bool isChapterUnlocked(int ch) {
 if (ch < 1 || ch > MAX_CHAPTERS_PER_CHARACTER) return false;
 CharacterXPData* xp = getCurrentCharacterXP();
 return xp ? xp->level >= CHAPTER_LEVELS[ch-1] : ch == 1;
}
int getUnlockedChapterCount() { int c = 0; for (int i = 0; i < MAX_CHAPTERS_PER_CHARACTER; i++) if (isChapterUnlocked(i+1)) c++; return c; }

void startChapter(int ch) {
 if (!story_system.current_story || ch < 1 || ch > MAX_CHAPTERS_PER_CHARACTER || !isChapterUnlocked(ch)) return;
 story_system.current_story->current_chapter = ch;
 story_system.current_dialogue_index = 0;
 story_system.in_story_mode = true;
 story_system.in_boss_battle = false;
 story_system.showing_rewards = false;
 saveStoryProgressForCharacter(story_system.current_story->character);
}

void advanceDialogue() {
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter;
 if (ch < 1) return;
 StoryChapter* chapter = &story_system.current_story->chapters[ch-1];
 story_system.current_dialogue_index++;
 if (story_system.current_dialogue_index >= chapter->dialogue_count) {
   if (!chapter->boss_defeated) startStoryBoss();
   else story_system.showing_rewards = true;
 }
}

void handleDialogueChoice(int choice) {
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter;
 DialogueScreen* d = &story_system.current_story->chapters[ch-1].dialogues[story_system.current_dialogue_index];
 if (!d->has_choices) { advanceDialogue(); return; }
 if (story_system.current_story->character == THEME_YUGO_WAKFU && isYugoDecisionPoint()) {
   setYugoPath(choice == 1 ? YUGO_PATH_DRAGONS : YUGO_PATH_PORTALS);
 }
 story_system.current_dialogue_index = (choice == 1) ? d->choice1_next_index : d->choice2_next_index;
}

void completeCurrentChapter() {
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter;
 if (ch < 1) return;
 StoryChapter* chapter = &story_system.current_story->chapters[ch-1];
 if (!chapter->completed) {
   chapter->completed = true;
   story_system.current_story->chapters_completed++;
   if (story_system.current_story->chapters_completed >= MAX_CHAPTERS_PER_CHARACTER) story_system.current_story->story_completed = true;
   gainExperience(XP_CHAPTER_COMPLETE, "Chapter Complete");
   system_state.player_gems += GEMS_CHAPTER_COMPLETE;
   saveStoryProgressForCharacter(story_system.current_story->character);
 }
}

// =============================================================================
// BOSS BATTLES
// =============================================================================
void startStoryBoss() {
 if (!story_system.current_story) return;
 story_system.in_boss_battle = true;
 CharacterXPData* xp = getCurrentCharacterXP();
 int lv = xp ? xp->level : 1;
 story_system.player_max_hp = 100 + lv * 10;
 story_system.player_hp = story_system.player_max_hp;
 StoryChapter* ch = &story_system.current_story->chapters[story_system.current_story->current_chapter - 1];
 story_system.boss_max_hp = ch->boss.hp;
 story_system.boss_hp = story_system.boss_max_hp;
}

void handleStoryBossAttack() {
 if (!story_system.in_boss_battle || !story_system.current_story) return;
 CharacterXPData* xp = getCurrentCharacterXP();
 int atk = 20 + (xp ? xp->level * 5 : 0);
 story_system.boss_hp -= atk;
 if (story_system.boss_hp > 0) {
   StoryChapter* ch = &story_system.current_story->chapters[story_system.current_story->current_chapter - 1];
   int dmg = max(1, ch->boss.attack - (xp ? xp->level : 0));
   story_system.player_hp -= dmg;
   if (story_system.player_hp <= 0) { story_system.player_hp = 0; story_system.in_boss_battle = false; story_system.in_story_mode = false; }
 } else {
   story_system.boss_hp = 0;
   story_system.current_story->chapters[story_system.current_story->current_chapter - 1].boss_defeated = true;
   story_system.in_boss_battle = false; story_system.showing_rewards = true;
   saveStoryProgressForCharacter(story_system.current_story->character);
 }
}

void handleStoryBossSpecial() {
 if (!story_system.in_boss_battle) return;
 CharacterXPData* xp = getCurrentCharacterXP();
 story_system.boss_hp -= 50 + (xp ? xp->level * 8 : 0);
 if (story_system.boss_hp <= 0) {
   story_system.boss_hp = 0;
   if (story_system.current_story) {
     story_system.current_story->chapters[story_system.current_story->current_chapter - 1].boss_defeated = true;
     saveStoryProgressForCharacter(story_system.current_story->character);
   }
   story_system.in_boss_battle = false; story_system.showing_rewards = true;
 }
}

bool isStoryBossDefeated() { return story_system.boss_hp <= 0; }

void claimBossRewards() {
 if (!story_system.current_story) return;
 StoryChapter* ch = &story_system.current_story->chapters[story_system.current_story->current_chapter - 1];
 if (!ch->rewards_claimed && ch->boss_defeated) {
   ch->rewards_claimed = true;
   gainExperience(ch->boss.xp_reward, "Boss Defeated");
   system_state.player_gems += ch->boss.gem_reward;
   completeCurrentChapter();
   story_system.showing_rewards = false; story_system.in_story_mode = false;
   saveStoryProgressForCharacter(story_system.current_story->character);
 }
}

// =============================================================================
// YUGO PATH
// =============================================================================
void setYugoPath(YugoStoryPath p) {
 if (stories[THEME_YUGO_WAKFU].yugo_path == YUGO_PATH_UNDECIDED) {
   stories[THEME_YUGO_WAKFU].yugo_path = p; loadYugoPathStory(p);
   if (story_system.nvs_initialized) story_system.prefs.putInt("yugo_path", (int)p);
 }
}
YugoStoryPath getYugoPath() { return stories[THEME_YUGO_WAKFU].yugo_path; }
bool isYugoDecisionPoint() {
 if (!story_system.current_story || story_system.current_story->character != THEME_YUGO_WAKFU) return false;
 return story_system.current_story->current_chapter == 3 && stories[THEME_YUGO_WAKFU].yugo_path == YUGO_PATH_UNDECIDED
   && story_system.current_dialogue_index >= story_system.current_story->chapters[2].dialogue_count - 1;
}
void loadYugoPathStory(YugoStoryPath p) {
 const char** t = (p == YUGO_PATH_DRAGONS) ? YUGO_DRAGON_TITLES : YUGO_PORTAL_TITLES;
 for (int i = 0; i < MAX_CHAPTERS_PER_CHARACTER; i++) stories[THEME_YUGO_WAKFU].chapters[i].title = t[i];
}

// =============================================================================
// EVENTS
// =============================================================================
void checkDailyStoryEvents() {
 WatchTime c = getCurrentTime();
 if (c.day != story_system.last_event_check_day) {
   for (int i = 0; i < 4; i++) story_system.daily_events[i].completed_today = false;
   story_system.last_event_check_day = c.day; saveStoryProgress();
 }
}

// =============================================================================
// CHAPTER UNLOCK NOTIFICATION SYSTEM
// =============================================================================
void checkChapterUnlocks() {
    if (!story_system.current_story) return;
    
    // Don't check if notification is already active
    if (story_system.notification.isActive) return;
    
    CharacterXPData* xp = getCurrentCharacterXP();
    int currentLevel = xp ? xp->level : 1;
    
    // Skip if level hasn't changed since last check
    if (currentLevel == story_system.lastCheckedLevel) return;
    
    // Check each chapter for new unlocks
    for (int i = 0; i < MAX_CHAPTERS_PER_CHARACTER; i++) {
        int levelRequired = CHAPTER_LEVELS[i];
        StoryChapter* chapter = &story_system.current_story->chapters[i];
        
        // If chapter should be unlocked based on level but wasn't before
        bool wasUnlocked = (story_system.lastCheckedLevel >= levelRequired);
        bool nowUnlocked = (currentLevel >= levelRequired);
        
        // If newly unlocked (and not the first chapter)
        if (nowUnlocked && !wasUnlocked && i > 0 && !chapter->completed) {
            // Update level FIRST so we don't re-trigger
            story_system.lastCheckedLevel = currentLevel;
            triggerChapterUnlock(i + 1);  // Chapter numbers are 1-indexed
            return;  // Only show one notification at a time
        }
    }
    
    story_system.lastCheckedLevel = currentLevel;
}

void triggerChapterUnlock(int chapterNum) {
    story_system.notification.isActive = true;
    story_system.notification.isPending = false;
    story_system.notification.chapterUnlocked = chapterNum;
    story_system.notification.startTime = millis();
    story_system.notification.userDismissed = false;
    story_system.notification.userAccepted = false;
    
    Serial.printf("[Story] Chapter %d UNLOCKED! Showing notification.\n", chapterNum);
}

void dismissNotification() {
    story_system.notification.isActive = false;
    story_system.notification.userDismissed = true;
    Serial.println("[Story] User chose 'Later' - notification dismissed");
}

void acceptNotification() {
    int chapterIdx = story_system.notification.chapterUnlocked - 1;
    story_system.notification.isActive = false;
    story_system.notification.userAccepted = true;
    
    // Go directly to the new chapter
    if (story_system.current_story && chapterIdx >= 0 && chapterIdx < MAX_CHAPTERS_PER_CHARACTER) {
        startChapter(story_system.notification.chapterUnlocked);
    }
    
    Serial.printf("[Story] User chose 'Ready' - opening chapter %d\n", 
                  story_system.notification.chapterUnlocked);
}

void updateNotificationTimeout() {
    if (!story_system.notification.isActive) return;
    
    if (millis() - story_system.notification.startTime > NOTIFICATION_TIMEOUT_MS) {
        dismissNotification();
    }
}

// =============================================================================
// CHAPTER UNLOCK NOTIFICATION DRAWING
// =============================================================================
void drawChapterUnlockNotification() {
    // Semi-transparent overlay
    for (int y = 0; y < LCD_HEIGHT; y += 2) {
        gfx->drawFastHLine(0, y, LCD_WIDTH, RGB565(5, 5, 10));
    }
    
    ThemeColors* theme = getCurrentTheme();
    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    
    // Notification panel
    int panelW = 320;
    int panelH = 200;
    int panelX = centerX - panelW/2;
    int panelY = centerY - panelH/2;
    
    // Panel background with glow
    gfx->fillRect(panelX - 4, panelY - 4, panelW + 8, panelH + 8, theme->primary);
    gfx->fillRect(panelX, panelY, panelW, panelH, RGB565(15, 18, 28));
    gfx->drawRect(panelX, panelY, panelW, panelH, theme->accent);
    
    // Corner decorations
    gfx->fillRect(panelX, panelY, 10, 10, theme->primary);
    gfx->fillRect(panelX + panelW - 10, panelY, 10, 10, theme->primary);
    gfx->fillRect(panelX, panelY + panelH - 10, 10, 10, theme->primary);
    gfx->fillRect(panelX + panelW - 10, panelY + panelH - 10, 10, 10, theme->primary);
    
    // Star/unlock icon
    gfx->setTextSize(4);
    gfx->setTextColor(COLOR_GOLD);
    gfx->setCursor(centerX - 12, panelY + 20);
    gfx->print("*");
    
    // Title
    gfx->setTextSize(2);
    gfx->setTextColor(theme->primary);
    gfx->setCursor(centerX - 96, panelY + 60);
    gfx->print("NEW CHAPTER");
    
    gfx->setTextSize(3);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(centerX - 72, panelY + 85);
    gfx->print("UNLOCKED!");
    
    // Chapter info
    if (story_system.current_story && story_system.notification.chapterUnlocked > 0) {
        int chapterIdx = story_system.notification.chapterUnlocked - 1;
        StoryChapter* chapter = &story_system.current_story->chapters[chapterIdx];
        
        gfx->setTextSize(2);
        gfx->setTextColor(theme->accent);
        gfx->setCursor(centerX - 60, panelY + 115);
        char chBuf[20]; sprintf(chBuf, "Chapter %d", chapter->chapter_number);
        gfx->print(chBuf);
        
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(200, 205, 220));
        int titleLen = strlen(chapter->title) * 6;
        gfx->setCursor(centerX - titleLen/2, panelY + 140);
        gfx->print(chapter->title);
    }
    
    // Buttons
    int btnW = 120;
    int btnH = 40;
    int btnY = panelY + panelH - 55;
    int btn1X = centerX - btnW - 10;
    int btn2X = centerX + 10;
    
    // "Nah Later" button
    gfx->fillRect(btn1X, btnY, btnW, btnH, RGB565(40, 40, 50));
    gfx->drawRect(btn1X, btnY, btnW, btnH, RGB565(80, 80, 90));
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565(180, 180, 190));
    gfx->setCursor(btn1X + 20, btnY + 12);
    gfx->print("LATER");
    
    // "Yes Ready" button
    gfx->fillRect(btn2X, btnY, btnW, btnH, theme->primary);
    gfx->drawRect(btn2X, btnY, btnW, btnH, theme->accent);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(btn2X + 15, btnY + 12);
    gfx->print("READY!");
    
    // Auto-dismiss timer
    unsigned long elapsed = millis() - story_system.notification.startTime;
    int remaining = (NOTIFICATION_TIMEOUT_MS - elapsed) / 1000 + 1;
    if (remaining > 0 && remaining <= 5) {
        gfx->setTextSize(1);
        gfx->setTextColor(RGB565(100, 100, 110));
        gfx->setCursor(centerX - 30, panelY + panelH - 15);
        char tmBuf[20]; sprintf(tmBuf, "Auto-close: %ds", remaining);
        gfx->print(tmBuf);
    }
}

void handleUnlockNotificationTouch(TouchGesture& gesture) {
    if (gesture.event != TOUCH_TAP) return;
    
    int x = gesture.x, y = gesture.y;
    int centerX = LCD_WIDTH / 2;
    int centerY = LCD_HEIGHT / 2;
    
    int btnW = 120;
    int btnH = 40;
    int panelH = 200;
    int panelY = centerY - panelH/2;
    int btnY = panelY + panelH - 55;
    int btn1X = centerX - btnW - 10;
    int btn2X = centerX + 10;
    
    // "Later" button
    if (x >= btn1X && x < btn1X + btnW && y >= btnY && y < btnY + btnH) {
        dismissNotification();
        drawStoryMenu();  // Refresh the menu
        return;
    }
    
    // "Ready" button
    if (x >= btn2X && x < btn2X + btnW && y >= btnY && y < btnY + btnH) {
        acceptNotification();
        return;
    }
}
StoryEvent* getCurrentActiveEvent() {
 WatchTime c = getCurrentTime();
 for (int i = 0; i < 4; i++) {
   StoryEvent* e = &story_system.daily_events[i];
   bool in_w = (e->start_hour <= e->end_hour) ? (c.hour >= e->start_hour && c.hour < e->end_hour) : (c.hour >= e->start_hour || c.hour < e->end_hour);
   if (in_w && !e->completed_today) return e;
 }
 return nullptr;
}
void claimEventReward(int idx) {
 if (idx < 0 || idx >= 4) return;
 StoryEvent* e = &story_system.daily_events[idx];
 if (!e->completed_today) { e->completed_today = true; gainExperience(e->xp_reward, e->title); saveStoryProgress(); }
}
bool hasActiveEvent() { return getCurrentActiveEvent() != nullptr; }

// =============================================================================
// DRAWING FUNCTIONS (same structure as before, now with boss sprites)
// =============================================================================
void drawStoryMenu() {
 // Check for auto-dismiss notification timeout
 updateNotificationTimeout();
 if (story_system.notification.isActive) {
   drawChapterUnlockNotification();
   return;
 }
 checkChapterUnlocks();
 if (story_system.notification.isActive) {
   drawChapterUnlockNotification();
   return;
 }
 
 gfx->fillScreen(RGB565(2, 2, 8));
 for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
   gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(4, 4, 10));
 }
 ThemeColors colors = *getThemeColors(system_state.current_theme);
 
 // Dramatic header
 gfx->fillRect(0, 0, LCD_WIDTH, 60, RGB565(8, 10, 16));
 gfx->fillRect(0, 56, LCD_WIDTH, 4, colors.primary);
 for (int i = 0; i < 6; i++) {
   gfx->drawLine(LCD_WIDTH - 60 + i*10, 0, LCD_WIDTH - 30 + i*10, 56, RGB565(20, 25, 35));
 }
 gfx->setTextColor(colors.primary); gfx->setTextSize(3);
 gfx->setCursor(20, 6); gfx->print("STORY");
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2);
 gfx->setCursor(20, 34); gfx->print("MODE");
 
 if (story_system.current_story) {
   gfx->fillRect(15, 72, 5, 30, colors.accent);
   gfx->setTextSize(2); gfx->setTextColor(colors.accent);
   gfx->setCursor(28, 76);
   gfx->print(story_system.current_story->story_name);
   
   int p = (story_system.current_story->chapters_completed * 100) / MAX_CHAPTERS_PER_CHARACTER;
   int barY = 110;
   gfx->fillRoundRect(20, barY, LCD_WIDTH-40, 22, 6, RGB565(15, 18, 25));
   gfx->drawRoundRect(20, barY, LCD_WIDTH-40, 22, 6, RGB565(40, 45, 60));
   int fillW = ((LCD_WIDTH-44)*p)/100;
   if (fillW > 0) {
     gfx->fillRoundRect(22, barY+2, fillW, 18, 4, colors.primary);
     gfx->drawFastHLine(22, barY+2, fillW, colors.accent);
   }
   gfx->setTextSize(1); gfx->setTextColor(COLOR_WHITE);
   gfx->setCursor(LCD_WIDTH/2-15, barY+6);
   char pt[10]; sprintf(pt, "%d%%", p); gfx->print(pt);
   gfx->setTextColor(colors.accent);
   gfx->setCursor(LCD_WIDTH - 90, barY+6);
   char chCnt[16]; sprintf(chCnt, "%d/%d Ch", getUnlockedChapterCount(), MAX_CHAPTERS_PER_CHARACTER);
   gfx->print(chCnt);
 }
 
 // Chapters button - manga panel style
 int btnY = 150;
 gfx->fillRoundRect(25, btnY, LCD_WIDTH-50, 55, 8, RGB565(15, 20, 30));
 gfx->drawRoundRect(25, btnY, LCD_WIDTH-50, 55, 8, colors.primary);
 gfx->fillRect(25, btnY, 6, 55, colors.primary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2);
 gfx->setCursor(45, btnY+8); gfx->print("CHAPTERS");
 gfx->setTextColor(colors.accent); gfx->setTextSize(1);
 gfx->setCursor(45, btnY+32); gfx->print("Continue your adventure");
 gfx->setTextColor(colors.primary); gfx->setTextSize(2);
 gfx->setCursor(LCD_WIDTH-55, btnY+18); gfx->print(">>");
 
 // Events button
 btnY = 225;
 gfx->fillRoundRect(25, btnY, LCD_WIDTH-50, 55, 8, RGB565(15, 20, 30));
 gfx->drawRoundRect(25, btnY, LCD_WIDTH-50, 55, 8, colors.secondary);
 gfx->fillRect(25, btnY, 6, 55, colors.secondary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2);
 gfx->setCursor(45, btnY+8); gfx->print("EVENTS");
 gfx->setTextColor(RGB565(120, 125, 140)); gfx->setTextSize(1);
 gfx->setCursor(45, btnY+32); gfx->print("Daily story challenges");
 gfx->setTextColor(colors.secondary); gfx->setTextSize(2);
 gfx->setCursor(LCD_WIDTH-55, btnY+18); gfx->print(">>");
 
 // Back button
 gfx->fillRoundRect(25, LCD_HEIGHT-65, 85, 42, 8, RGB565(15, 18, 25));
 gfx->drawRoundRect(25, LCD_HEIGHT-65, 85, 42, 8, RGB565(50, 55, 70));
 gfx->setTextColor(RGB565(150, 155, 170)); gfx->setTextSize(2);
 gfx->setCursor(38, LCD_HEIGHT-52); gfx->print("Back");
}

// Chapter select scroll offset (for 15 chapters)
static int chapterScrollOffset = 0;
#define CHAPTERS_VISIBLE 5

void drawChapterSelect() {
 gfx->fillScreen(RGB565(2, 2, 5));
 // CRT scan lines
 for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
   gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(4, 4, 7));
 }
 
 ThemeColors colors = *getThemeColors(system_state.current_theme);
 
 // Retro header
 int headerH = 55;
 gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
 for (int hx = 0; hx < LCD_WIDTH; hx += 8) {
   gfx->fillRect(hx, headerH - 3, 6, 3, colors.primary);
 }
 
 gfx->setTextSize(3);
 gfx->setTextColor(RGB565(30, 35, 50));
 gfx->setCursor(LCD_WIDTH/2 - 82, 14);
 gfx->print("Chapters");
 gfx->setTextColor(colors.primary);
 gfx->setCursor(LCD_WIDTH/2 - 84, 12);
 gfx->print("Chapters");
 
 // Page indicator
 gfx->setTextSize(2); gfx->setTextColor(colors.accent);
 gfx->setCursor(LCD_WIDTH - 90, 18);
 char pgBuf[16]; sprintf(pgBuf, "%d/%d", (chapterScrollOffset / CHAPTERS_VISIBLE) + 1, 
             (MAX_CHAPTERS_PER_CHARACTER + CHAPTERS_VISIBLE - 1) / CHAPTERS_VISIBLE);
 gfx->print(pgBuf);
 
 if (!story_system.current_story) return;
 int y_off = headerH + 10;
 int cardH = 72;
 int cardGap = 8;
 
 for (int i = 0; i < CHAPTERS_VISIBLE; i++) {
   int chIdx = i + chapterScrollOffset;
   if (chIdx >= MAX_CHAPTERS_PER_CHARACTER) break;
   
   StoryChapter* ch = &story_system.current_story->chapters[chIdx];
   bool unlocked = isChapterUnlocked(chIdx + 1);
   
   int cardY = y_off + i * (cardH + cardGap);
   
   // Card background
   uint16_t cardBg = ch->completed ? RGB565(15, 25, 15) : (unlocked ? RGB565(15, 18, 25) : RGB565(12, 12, 15));
   gfx->fillRect(20, cardY, LCD_WIDTH-40, cardH, cardBg);
   
   // Card border
   uint16_t borderColor = ch->completed ? RGB565(0, 200, 80) : (unlocked ? colors.primary : RGB565(40, 40, 50));
   gfx->drawRect(20, cardY, LCD_WIDTH-40, cardH, borderColor);
   
   // Corner accents
   gfx->fillRect(20, cardY, 5, 5, borderColor);
   gfx->fillRect(LCD_WIDTH - 25, cardY, 5, 5, borderColor);
   
   // Left color stripe
   gfx->fillRect(20, cardY + 2, 5, cardH - 4, borderColor);
   
   // Chapter number - big
   gfx->setTextColor(unlocked ? colors.primary : RGB565(60, 60, 70));
   gfx->setTextSize(3);
   gfx->setCursor(35, cardY + 10);
   char chNum[5]; sprintf(chNum, "%02d", chIdx + 1);
   gfx->print(chNum);
   
   // Chapter title
   gfx->setTextColor(unlocked ? COLOR_WHITE : RGB565(80, 80, 90));
   gfx->setTextSize(2);
   gfx->setCursor(80, cardY + 12);
   // Truncate long titles
   char titleBuf[20];
   strncpy(titleBuf, ch->title, 19);
   titleBuf[19] = '\0';
   gfx->print(titleBuf);
   
   // Status text
   if (ch->completed) { 
     gfx->setTextColor(RGB565(0, 200, 80));
     gfx->setTextSize(2);
     gfx->setCursor(80, cardY + 42);
     gfx->print("COMPLETE");
   }
   else if (!unlocked) { 
     gfx->setTextColor(RGB565(200, 60, 60));
     gfx->setTextSize(2);
     gfx->setCursor(80, cardY + 42);
     gfx->print("Lv."); gfx->print(CHAPTER_LEVELS[chIdx]);
     gfx->setTextColor(RGB565(80, 80, 90));
     gfx->print(" needed");
   }
   else {
     gfx->setTextColor(RGB565(0, 200, 255));
     gfx->setTextSize(2);
     gfx->setCursor(80, cardY + 42);
     gfx->print(">> PLAY");
   }
 }
 
 // Scroll arrows area
 int arrowY = y_off + CHAPTERS_VISIBLE * (cardH + cardGap) + 5;
 
 if (chapterScrollOffset > 0) {
   gfx->fillRect(30, arrowY, 80, 30, RGB565(15, 18, 25));
   gfx->drawRect(30, arrowY, 80, 30, colors.accent);
   gfx->setTextSize(2); gfx->setTextColor(colors.accent);
   gfx->setCursor(50, arrowY + 7); gfx->print("UP");
 }
 
 if (chapterScrollOffset + CHAPTERS_VISIBLE < MAX_CHAPTERS_PER_CHARACTER) {
   gfx->fillRect(LCD_WIDTH - 110, arrowY, 80, 30, RGB565(15, 18, 25));
   gfx->drawRect(LCD_WIDTH - 110, arrowY, 80, 30, colors.accent);
   gfx->setTextSize(2); gfx->setTextColor(colors.accent);
   gfx->setCursor(LCD_WIDTH - 96, arrowY + 7); gfx->print("DOWN");
 }
 
 // Back button - retro style
 gfx->fillRect(30, LCD_HEIGHT - 55, 80, 38, RGB565(15, 18, 25));
 gfx->drawRect(30, LCD_HEIGHT - 55, 80, 38, RGB565(40, 45, 60));
 gfx->fillRect(30, LCD_HEIGHT - 55, 5, 5, colors.primary);
 gfx->setTextColor(RGB565(180, 185, 200)); gfx->setTextSize(2); gfx->setCursor(45, LCD_HEIGHT - 44); gfx->print("Back");
 
 drawSwipeIndicator();
}

void drawDialogueScreen() {
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter; if (ch < 1) return;
 StoryChapter* chapter = &story_system.current_story->chapters[ch-1];
 DialogueScreen* d = &chapter->dialogues[story_system.current_dialogue_index];
 
 gfx->fillScreen(RGB565(5, 5, 10));
 // CRT scan lines
 for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
   gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(8, 8, 14));
 }
 
 ThemeColors colors = *getThemeColors(system_state.current_theme);
 
 // Header bar with chapter info
 int headerH = 55;
 gfx->fillRect(0, 0, LCD_WIDTH, headerH, RGB565(10, 12, 18));
 for (int hx = 0; hx < LCD_WIDTH; hx += 8) {
   gfx->fillRect(hx, headerH - 3, 6, 3, colors.primary);
 }
 
 // Chapter title - large and visible
 gfx->setTextColor(colors.primary);
 gfx->setTextSize(2);
 gfx->setCursor(20, 8);
 gfx->print("Ch.");
 gfx->print(ch);
 
 gfx->setTextColor(COLOR_WHITE);
 gfx->setTextSize(2);
 gfx->setCursor(20, 30);
 // Truncate title if too long
 char titleBuf[25];
 strncpy(titleBuf, chapter->title, 24);
 titleBuf[24] = '\0';
 gfx->print(titleBuf);
 
 // Dialogue progress indicator
 gfx->setTextColor(RGB565(100, 105, 120));
 gfx->setTextSize(1);
 gfx->setCursor(LCD_WIDTH - 60, 20);
 char progBuf[10];
 sprintf(progBuf, "%d/%d", story_system.current_dialogue_index + 1, chapter->dialogue_count);
 gfx->print(progBuf);
 
 // Scene area (top half) - themed background
 int sceneY = headerH + 5;
 int sceneH = 150;
 gfx->fillRect(15, sceneY, LCD_WIDTH - 30, sceneH, RGB565(12, 14, 20));
 gfx->drawRect(15, sceneY, LCD_WIDTH - 30, sceneH, RGB565(35, 40, 55));
 gfx->fillRect(15, sceneY, 6, 6, colors.primary);
 gfx->fillRect(LCD_WIDTH - 21, sceneY, 6, 6, colors.primary);
 
 // Character name in scene area
 gfx->setTextColor(d->speaker_color);
 gfx->setTextSize(3);
 int nameLen = strlen(d->speaker) * 18;
 gfx->setCursor((LCD_WIDTH - nameLen) / 2, sceneY + 55);
 gfx->print(d->speaker);
 
 // Dialogue box - bottom area
 int dlgY = sceneY + sceneH + 15;
 int dlgH = LCD_HEIGHT - dlgY - 50;
 gfx->fillRoundRect(15, dlgY, LCD_WIDTH - 30, dlgH, 10, RGB565(20, 22, 30));
 gfx->drawRoundRect(15, dlgY, LCD_WIDTH - 30, dlgH, 10, colors.primary);
 gfx->fillRect(15, dlgY, 6, 6, colors.accent);
 gfx->fillRect(LCD_WIDTH - 21, dlgY, 6, 6, colors.accent);
 
 // Speaker name in dialogue box
 gfx->setTextColor(d->speaker_color);
 gfx->setTextSize(2);
 gfx->setCursor(30, dlgY + 12);
 gfx->print(d->speaker);
 
 // Separator line under name
 gfx->drawFastHLine(30, dlgY + 34, LCD_WIDTH - 80, RGB565(50, 55, 70));
 
 // Dialogue text - SIZE 2 for readability!
 gfx->setTextColor(COLOR_WHITE);
 gfx->setTextSize(2);
 const char* txt = d->text;
 int tx = 30, ty = dlgY + 45;
 int maxTextX = LCD_WIDTH - 50;
 
 while (*txt) {
   char w[50]; int wl = 0;
   while (*txt && *txt != ' ' && wl < 49) w[wl++] = *txt++;
   w[wl] = 0;
   int wordPixelW = wl * 12; // size 2 = 12px per char
   if (tx + wordPixelW > maxTextX) { tx = 30; ty += 22; }
   if (ty > dlgY + dlgH - 20) break; // Don't overflow
   gfx->setCursor(tx, ty);
   gfx->print(w);
   tx += wordPixelW + 12;
   if (*txt == ' ') txt++;
 }
 
 if (d->has_choices) {
   drawChoiceScreen(d->choice1, d->choice2);
 } else {
   // Tap to continue indicator
   gfx->setTextColor(RGB565(80, 85, 100));
   gfx->setTextSize(2);
   gfx->setCursor(LCD_WIDTH/2 - 84, LCD_HEIGHT - 35);
   gfx->print("Tap to continue");
   // Blinking arrow
   gfx->setTextColor(colors.primary);
   gfx->setCursor(LCD_WIDTH - 40, LCD_HEIGHT - 35);
   gfx->print(">>");
 }
}

void drawChoiceScreen(const char* c1, const char* c2) {
 ThemeColors colors = *getThemeColors(system_state.current_theme);  // FIX: Added * to dereference pointer
 gfx->fillRoundRect(30, LCD_HEIGHT-180, LCD_WIDTH-60, 60, 10, colors.primary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2); gfx->setCursor(50, LCD_HEIGHT-160); gfx->print("1: ");
 gfx->setTextSize(1); gfx->print(c1);
 gfx->fillRoundRect(30, LCD_HEIGHT-100, LCD_WIDTH-60, 60, 10, colors.secondary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2); gfx->setCursor(50, LCD_HEIGHT-80); gfx->print("2: ");
 gfx->setTextSize(1); gfx->print(c2);
}

void drawStoryBossScreen() {
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter;
 StoryChapter* chapter = &story_system.current_story->chapters[ch-1];
 StoryBoss* boss = &chapter->boss;
 
 gfx->fillScreen(RGB565(5, 2, 2));
 // Battle scan lines
 for (int sy = 0; sy < LCD_HEIGHT; sy += 3) {
   gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(8, 3, 3));
 }
 ThemeColors colors = *getThemeColors(system_state.current_theme);

 // Boss name header - dramatic red gradient bar
 gfx->fillRect(0, 0, LCD_WIDTH, 55, RGB565(15, 5, 5));
 gfx->fillRect(0, 51, LCD_WIDTH, 4, boss->boss_color);
 gfx->setTextColor(boss->boss_color); gfx->setTextSize(3);
 int nameLen = strlen(boss->name) * 18;
 gfx->setCursor((LCD_WIDTH - nameLen) / 2, 6);
 gfx->print(boss->name);
 gfx->setTextColor(boss->boss_accent); gfx->setTextSize(1);
 int serLen = strlen(boss->series) * 6;
 gfx->setCursor((LCD_WIDTH - serLen) / 2, 38);
 gfx->print(boss->series);

 // Boss sprite with aura glow
 static int boss_anim = 0; boss_anim++;
 // Aura rings behind boss
 for (int r = 55; r >= 40; r -= 5) {
   gfx->drawCircle(LCD_WIDTH/2, 140, r, boss->boss_color);
 }
 drawBossSprite(LCD_WIDTH/2, 140, boss, 50, boss_anim);

 // Boss HP bar - styled
 int barY = 210;
 gfx->fillRoundRect(30, barY, LCD_WIDTH-60, 26, 6, RGB565(15, 8, 8));
 gfx->drawRoundRect(30, barY, LCD_WIDTH-60, 26, 6, RGB565(60, 30, 30));
 int bhpw = (story_system.boss_hp * (LCD_WIDTH-64)) / story_system.boss_max_hp;
 if (bhpw > 0) {
   gfx->fillRoundRect(32, barY+2, max(4, bhpw), 22, 4, COLOR_RED);
   gfx->drawFastHLine(32, barY+2, max(4, bhpw), RGB565(255, 100, 100));
 }
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(1);
 gfx->setCursor(LCD_WIDTH/2-30, barY+8);
 char hp[20]; sprintf(hp, "%d/%d", story_system.boss_hp, story_system.boss_max_hp); gfx->print(hp);

 // "VS" divider
 gfx->setTextColor(COLOR_RED); gfx->setTextSize(2);
 gfx->setCursor(LCD_WIDTH/2-12, barY+35);
 gfx->print("VS");

 // Player HP bar
 int pbarY = barY + 60;
 gfx->fillRoundRect(30, pbarY, LCD_WIDTH-60, 26, 6, RGB565(8, 15, 8));
 gfx->drawRoundRect(30, pbarY, LCD_WIDTH-60, 26, 6, RGB565(30, 60, 30));
 int phpw = (story_system.player_hp * (LCD_WIDTH-64)) / story_system.player_max_hp;
 if (phpw > 0) {
   gfx->fillRoundRect(32, pbarY+2, max(4, phpw), 22, 4, COLOR_GREEN);
   gfx->drawFastHLine(32, pbarY+2, max(4, phpw), RGB565(100, 255, 100));
 }
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(1);
 gfx->setCursor(LCD_WIDTH/2-30, pbarY+8);
 sprintf(hp, "%d/%d", story_system.player_hp, story_system.player_max_hp); gfx->print(hp);
 gfx->setTextColor(colors.accent); gfx->setTextSize(1);
 gfx->setCursor(35, pbarY-12);
 gfx->print("YOUR HP");

 // Attack button - bold manga style
 int btnY2 = LCD_HEIGHT-115;
 gfx->fillRoundRect(20, btnY2, 170, 55, 10, colors.primary);
 gfx->drawRoundRect(20, btnY2, 170, 55, 10, colors.accent);
 gfx->fillRect(20, btnY2, 6, 55, colors.accent);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2);
 gfx->setCursor(50, btnY2+18); gfx->print("ATTACK");
 
 // Special button
 gfx->fillRoundRect(LCD_WIDTH-190, btnY2, 170, 55, 10, colors.accent);
 gfx->drawRoundRect(LCD_WIDTH-190, btnY2, 170, 55, 10, colors.primary);
 gfx->fillRect(LCD_WIDTH-24, btnY2, 6, 55, colors.primary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2);
 gfx->setCursor(LCD_WIDTH-170, btnY2+18); gfx->print("SPECIAL");
 
 // Flee button
 gfx->fillRoundRect(LCD_WIDTH/2-45, LCD_HEIGHT-48, 90, 36, 8, RGB565(20, 20, 25));
 gfx->drawRoundRect(LCD_WIDTH/2-45, LCD_HEIGHT-48, 90, 36, 8, RGB565(60, 60, 70));
 gfx->setTextColor(RGB565(120, 120, 130)); gfx->setTextSize(1);
 gfx->setCursor(LCD_WIDTH/2-12, LCD_HEIGHT-36);
 gfx->print("Flee");
}

void drawChapterRewards() {
 if (!story_system.current_story) return;
 StoryChapter* ch = &story_system.current_story->chapters[story_system.current_story->current_chapter - 1];
 
 gfx->fillScreen(RGB565(2, 2, 8));
 for (int sy = 0; sy < LCD_HEIGHT; sy += 4) {
   gfx->drawFastHLine(0, sy, LCD_WIDTH, RGB565(5, 5, 12));
 }
 ThemeColors colors = *getThemeColors(system_state.current_theme);
 
 // Victory banner - dramatic diagonal lines
 gfx->fillRect(0, 0, LCD_WIDTH, 80, RGB565(10, 8, 2));
 for (int i = 0; i < 8; i++) {
   gfx->drawLine(i*60, 0, i*60 - 40, 80, RGB565(30, 25, 5));
 }
 gfx->fillRect(0, 76, LCD_WIDTH, 4, COLOR_GOLD);
 
 // "VICTORY" text with shadow
 gfx->setTextColor(RGB565(40, 35, 10)); gfx->setTextSize(4);
 gfx->setCursor(LCD_WIDTH/2 - 80, 18);
 gfx->print("VICTORY");
 gfx->setTextColor(COLOR_GOLD); gfx->setTextSize(4);
 gfx->setCursor(LCD_WIDTH/2 - 82, 16);
 gfx->print("VICTORY");
 // Subtitle
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(1);
 gfx->setCursor(LCD_WIDTH/2 - 50, 58);
 gfx->print(ch->boss.name);
 gfx->print(" defeated!");
 
 // Rewards panel
 int panelY = 100;
 gfx->fillRoundRect(25, panelY, LCD_WIDTH-50, 180, 10, RGB565(12, 14, 22));
 gfx->drawRoundRect(25, panelY, LCD_WIDTH-50, 180, 10, colors.primary);
 gfx->fillRect(25, panelY, 6, 180, colors.primary);
 
 gfx->setTextColor(colors.primary); gfx->setTextSize(2);
 gfx->setCursor(45, panelY+12); gfx->print("REWARDS");
 gfx->drawFastHLine(45, panelY+34, LCD_WIDTH-100, RGB565(30, 35, 50));
 
 // XP reward with icon
 gfx->setTextColor(COLOR_CYAN); gfx->setTextSize(2);
 gfx->setCursor(45, panelY+48);
 gfx->print("XP  +"); gfx->print(ch->boss.xp_reward + XP_CHAPTER_COMPLETE);
 
 // Gems reward
 gfx->setTextColor(COLOR_YELLOW); gfx->setTextSize(2);
 gfx->setCursor(45, panelY+78);
 gfx->print("GEM +"); gfx->print(ch->boss.gem_reward + GEMS_CHAPTER_COMPLETE);
 
 // Exclusive card
 if (ch->boss.exclusive_card_id >= 0) {
   gfx->fillRoundRect(40, panelY+110, LCD_WIDTH-80, 30, 6, RGB565(20, 12, 25));
   gfx->drawRoundRect(40, panelY+110, LCD_WIDTH-80, 30, 6, COLOR_PURPLE);
   gfx->setTextColor(COLOR_PURPLE); gfx->setTextSize(2);
   gfx->setCursor(55, panelY+116); gfx->print("EXCLUSIVE CARD!");
 }
 
 // Level info
 gfx->setTextColor(RGB565(100, 105, 120)); gfx->setTextSize(1);
 gfx->setCursor(45, panelY+152);
 gfx->print("Lv."); gfx->print(system_state.player_level);
 gfx->print(" | "); gfx->print(system_state.player_gems); gfx->print(" gems");
 
 // CLAIM button - large, glowing, unmissable
 int claimY = LCD_HEIGHT - 90;
 gfx->fillRoundRect(50, claimY, LCD_WIDTH-100, 60, 12, colors.primary);
 gfx->drawRoundRect(50, claimY, LCD_WIDTH-100, 60, 12, colors.accent);
 gfx->drawRoundRect(52, claimY+2, LCD_WIDTH-104, 56, 10, colors.accent);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(3);
 int claimTxtW = 5 * 18; // "CLAIM" = 5 chars
 gfx->setCursor((LCD_WIDTH - claimTxtW) / 2, claimY + 16);
 gfx->print("CLAIM");
}

void drawStoryEventPopup() {
 StoryEvent* e = getCurrentActiveEvent(); if (!e) return;
 ThemeColors c = *getThemeColors(system_state.current_theme);  // FIX: Added * to dereference pointer
 gfx->fillRoundRect(40, 100, LCD_WIDTH-80, 200, 15, RGB565(30,30,40));
 gfx->drawRoundRect(40, 100, LCD_WIDTH-80, 200, 15, c.accent);
 gfx->setTextColor(c.accent); gfx->setTextSize(2); gfx->setCursor(60, 120); gfx->print(e->title);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(1); gfx->setCursor(60, 160); gfx->print(e->description);
 gfx->setTextColor(COLOR_CYAN); gfx->setCursor(60, 200); gfx->print("Reward: +"); gfx->print(e->xp_reward); gfx->print(" XP");
 gfx->fillRoundRect(70, 240, LCD_WIDTH-140, 45, 10, c.primary);
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2); gfx->setCursor(LCD_WIDTH/2-40, 252); gfx->print("Claim!");
}

void drawYugoPathChoice() {
 gfx->fillScreen(COLOR_BLACK);
 gfx->setTextColor(YUGO_PORTAL_CYAN); gfx->setTextSize(2); gfx->setCursor(LCD_WIDTH/2-100, 50); gfx->print("Choose Your Path");
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(1); gfx->setCursor(LCD_WIDTH/2-120, 90); gfx->print("This choice will affect your story...");
 gfx->fillRoundRect(30, 140, LCD_WIDTH-60, 100, 15, RGB565(180,80,30));
 gfx->setTextColor(COLOR_WHITE); gfx->setTextSize(2); gfx->setCursor(50, 160); gfx->print("Dragon Path");
 gfx->setTextSize(1); gfx->setCursor(50, 200); gfx->print("Embrace dragon power - combat focused");
 gfx->fillRoundRect(30, 260, LCD_WIDTH-60, 100, 15, YUGO_PORTAL_CYAN);
 gfx->setTextColor(COLOR_BLACK); gfx->setTextSize(2); gfx->setCursor(50, 280); gfx->print("Portal Path");
 gfx->setTextSize(1); gfx->setCursor(50, 320); gfx->print("Master dimensions - exploration focused");
}

// =============================================================================
// TOUCH HANDLING
// =============================================================================
void handleStoryMenuTouch(TouchGesture& g) {
 if (g.event != TOUCH_TAP) return;
 
 // Handle notification touch first if active
 if (story_system.notification.isActive) {
   handleUnlockNotificationTouch(g);
   return;
 }
 
 if (g.y >= 150 && g.y <= 200) { chapterScrollOffset = 0; system_state.current_screen = SCREEN_CHAPTER_SELECT; drawChapterSelect(); return; }
 if (g.y >= 220 && g.y <= 270 && hasActiveEvent()) { drawStoryEventPopup(); return; }
 if (g.y >= LCD_HEIGHT-70 && g.x <= 110) { story_system.in_story_mode = false; returnToAppGrid(); }
}

void handleChapterSelectTouch(TouchGesture& g) {
 if (g.event == TOUCH_SWIPE_UP) {
   if (chapterScrollOffset + CHAPTERS_VISIBLE < MAX_CHAPTERS_PER_CHARACTER) {
     chapterScrollOffset += CHAPTERS_VISIBLE;
     drawChapterSelect();
   }
   return;
 }
 if (g.event == TOUCH_SWIPE_DOWN) {
   if (chapterScrollOffset > 0) {
     chapterScrollOffset -= CHAPTERS_VISIBLE;
     if (chapterScrollOffset < 0) chapterScrollOffset = 0;
     drawChapterSelect();
   }
   return;
 }
 
 if (g.event != TOUCH_TAP) return;
 
 int headerH = 55;
 int y_off = headerH + 10;
 int cardH = 72;
 int cardGap = 8;
 
 // Check chapter card taps
 for (int i = 0; i < CHAPTERS_VISIBLE; i++) {
   int chIdx = i + chapterScrollOffset;
   if (chIdx >= MAX_CHAPTERS_PER_CHARACTER) break;
   
   int cardY = y_off + i * (cardH + cardGap);
   if (g.y >= cardY && g.y <= cardY + cardH && g.x >= 20 && g.x <= LCD_WIDTH-20 && isChapterUnlocked(chIdx + 1)) {
     startChapter(chIdx + 1); system_state.current_screen = SCREEN_STORY_DIALOGUE; drawDialogueScreen(); return;
   }
 }
 
 // Scroll button taps
 int arrowY = y_off + CHAPTERS_VISIBLE * (cardH + cardGap) + 5;
 if (g.y >= arrowY && g.y <= arrowY + 30) {
   if (g.x < LCD_WIDTH/2 && chapterScrollOffset > 0) {
     chapterScrollOffset -= CHAPTERS_VISIBLE;
     if (chapterScrollOffset < 0) chapterScrollOffset = 0;
     drawChapterSelect();
     return;
   }
   if (g.x >= LCD_WIDTH/2 && chapterScrollOffset + CHAPTERS_VISIBLE < MAX_CHAPTERS_PER_CHARACTER) {
     chapterScrollOffset += CHAPTERS_VISIBLE;
     drawChapterSelect();
     return;
   }
 }
 
 // Back button
 if (g.y >= LCD_HEIGHT-55 && g.x <= 110) { system_state.current_screen = SCREEN_STORY_MENU; drawStoryMenu(); return; }
}

void handleDialogueTouch(TouchGesture& g) {
 if (g.event != TOUCH_TAP) return;
 if (isYugoDecisionPoint()) {
   if (g.y >= 140 && g.y <= 240) { setYugoPath(YUGO_PATH_DRAGONS); advanceDialogue(); return; }
   if (g.y >= 260 && g.y <= 360) { setYugoPath(YUGO_PATH_PORTALS); advanceDialogue(); return; }
   return;
 }
 if (!story_system.current_story) return;
 int ch = story_system.current_story->current_chapter; if (ch < 1) return;
 DialogueScreen* d = &story_system.current_story->chapters[ch-1].dialogues[story_system.current_dialogue_index];
 if (d->has_choices) {
   if (g.y >= LCD_HEIGHT-180 && g.y <= LCD_HEIGHT-120) handleDialogueChoice(1);
   else if (g.y >= LCD_HEIGHT-100 && g.y <= LCD_HEIGHT-40) handleDialogueChoice(2);
 } else {
   advanceDialogue();
   if (story_system.in_boss_battle) { system_state.current_screen = SCREEN_STORY_BOSS; drawStoryBossScreen(); }
   else if (story_system.showing_rewards) { drawChapterRewards(); }
   else { drawDialogueScreen(); }
 }
}

void handleStoryBossTouch(TouchGesture& g) {
 if (g.event != TOUCH_TAP) return;
 // PRIORITY: If showing rewards, ONLY handle claim button (no attack/special overlap)
 if (story_system.showing_rewards) {
   if (g.y >= LCD_HEIGHT-120 && g.y <= LCD_HEIGHT-40) {
     claimBossRewards();
     system_state.current_screen = SCREEN_STORY_MENU;
     drawStoryMenu();
   }
   return;
 }
 if (g.y >= LCD_HEIGHT-110 && g.y <= LCD_HEIGHT-60) {
   if (g.x <= 180) { handleStoryBossAttack(); if (!story_system.in_boss_battle && story_system.showing_rewards) drawChapterRewards(); else drawStoryBossScreen(); return; }
   if (g.x >= LCD_WIDTH-180) { handleStoryBossSpecial(); if (!story_system.in_boss_battle && story_system.showing_rewards) drawChapterRewards(); else drawStoryBossScreen(); return; }
 }
 if (g.y >= LCD_HEIGHT-50 && g.x >= LCD_WIDTH/2-50 && g.x <= LCD_WIDTH/2+50) {
   story_system.in_boss_battle = false; story_system.in_story_mode = false; system_state.current_screen = SCREEN_STORY_MENU; drawStoryMenu(); return;
 }
}

// =============================================================================
// HELPERS
// =============================================================================
uint16_t getStoryThemeColor(ThemeType t) { return getThemeColors(t)->primary; }  // This one uses -> correctly since it returns a pointer
const char* getChapterStatusText(int ch) {
 if (!story_system.current_story || ch < 1 || ch > MAX_CHAPTERS_PER_CHARACTER) return "Unknown";
 StoryChapter* c = &story_system.current_story->chapters[ch-1];
 if (c->completed) return "Complete"; if (!isChapterUnlocked(ch)) return "Locked";
 if (c->boss_defeated) return "Boss Cleared"; return "Available";
}
float getChapterProgress() {
 if (!story_system.current_story) return 0.0f;
 return (float)story_system.current_story->chapters_completed / (float)MAX_CHAPTERS_PER_CHARACTER;
}
