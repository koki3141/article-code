# メモ
## 仮想環境の作成方法

1. **Juliaを起動**:
   ```bash
   julia
   ```

2. **環境をアクティブ化**:
   ```julia
   using Pkg
   Pkg.activate(".")
   ```

3. **依存関係のインストール**:
   ```julia
   Pkg.instantiate()
   ```