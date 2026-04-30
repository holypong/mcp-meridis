# mcp-meridis — Claude Code ルール

## ファイル出力先

| 種別 | 保存先 | 例 |
|---|---|---|
| ログ（CSV） | `log/` | `log/p7_baseline.csv`, `log/buf_input.csv` |
| グラフ・レポート画像 | `report/` | `report/p7_gain_sweep.png` |

- `buf_input.csv` / `buf_output.csv` はシステムが使用する固定ファイル（プロジェクトルート）。それ以外の実験用 CSV はすべて `log/` に保存すること。
- `log/` と `report/` は `.gitignore` 対象のため、git には追跡されない。

## ログ削除のルール

- 「ログを削除して」と指示された場合は **`log/` ディレクトリ内の CSV ファイルのみ** を削除する。
- ドキュメント、`buf_input.csv` / `buf_output.csv`、コードファイルは削除しない。
