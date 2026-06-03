// 由 scripts/generateAiDocsSidebar.js 自动生成
// 请勿手动编辑 — 运行 node scripts/generateAiDocsSidebar.js 以更新

const aiDocsSidebar = [
  {
    "type": "category",
    "label": "关于",
    "collapsible": true,
    "collapsed": false,
    "items": [],
    "customProps": {
      "icon": "default-icons/ai-doc.svg",
      "tags": []
    },
    "link": {
      "type": "doc",
      "id": "关于/index"
    }
  },
  {
    "type": "category",
    "label": "知识沉淀",
    "collapsible": true,
    "collapsed": false,
    "items": [
      {
        "type": "category",
        "label": "现代C++",
        "collapsible": true,
        "collapsed": false,
        "items": [
          {
            "type": "category",
            "label": "日常探索",
            "collapsible": true,
            "collapsed": false,
            "items": [
              {
                "type": "category",
                "label": "HXLibs编写串行协程调度器",
                "collapsible": true,
                "collapsed": false,
                "items": [
                  {
                    "type": "doc",
                    "id": "知识沉淀/现代C++/日常探索/HXLibs编写串行协程调度器/.hx-mitemite"
                  }
                ],
                "customProps": {
                  "icon": "default-icons/ai-folder.svg",
                  "tags": []
                },
                "link": {
                  "type": "doc",
                  "id": "知识沉淀/现代C++/日常探索/HXLibs编写串行协程调度器/index"
                }
              }
            ],
            "customProps": {
              "icon": "default-icons/ai-folder.svg",
              "tags": []
            }
          }
        ],
        "customProps": {
          "icon": "default-icons/ai-folder.svg",
          "tags": []
        }
      }
    ],
    "customProps": {
      "icon": "default-icons/ai-folder.svg",
      "tags": []
    }
  }
];

export default { aiDocsSidebar };
